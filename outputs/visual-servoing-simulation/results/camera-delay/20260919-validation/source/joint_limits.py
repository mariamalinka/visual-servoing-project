"""Bounded joint-space least squares and progress monitoring for visual control."""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np

CONFIG_PATH = Path(__file__).resolve().parent / "joint_limit_config.json"


def load_joint_limit_config():
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8-sig"))


def bounded_least_squares(A, b, lower, upper, damping, preferred=None):
    """Minimize ||A v-b||^2 + damping^2 ||v-preferred||^2 inside a box.

    A small primal active-set solver: solve free variables, step to blocking
    bounds, and release bounds with incorrect gradient signs. No extra runtime
    dependency is required. Positive damping makes the Hessian definite.
    """
    A=np.asarray(A,dtype=float)
    b=np.asarray(b,dtype=float)
    lo=np.asarray(lower,dtype=float)
    hi=np.asarray(upper,dtype=float)
    if A.ndim!=2 or b.shape!=(A.shape[0],) or lo.shape!=(A.shape[1],) or hi.shape!=lo.shape:
        raise ValueError("Incompatible least-squares shapes")
    if not all(np.isfinite(x).all() for x in (A,b,lo,hi)) or np.any(lo>hi):
        raise ValueError("Expected finite least-squares inputs and ordered bounds")
    if not np.isfinite(damping) or damping<=0:
        raise ValueError("Damping must be finite and positive")
    prefer=np.zeros(len(lo)) if preferred is None else np.asarray(preferred,dtype=float)
    if prefer.shape!=lo.shape or not np.isfinite(prefer).all():
        raise ValueError("Expected a finite preferred velocity")
    H=A.T@A+damping*damping*np.eye(len(lo))
    g=-A.T@b-damping*damping*prefer
    unconstrained=np.linalg.solve(H,-g)
    x=np.clip(unconstrained,lo,hi)
    state=np.zeros(len(x),dtype=int)
    state[unconstrained<lo]=-1
    state[unconstrained>hi]=1
    fixed=lo==hi
    state[fixed]=2
    tolerance=1e-10*max(1.,float(np.linalg.norm(g,np.inf)))
    for _ in range(100):
        free=state==0
        candidate=x.copy()
        if free.any():
            bound=~free
            candidate[free]=np.linalg.solve(H[np.ix_(free,free)],
                -g[free]-H[np.ix_(free,bound)]@x[bound])
            direction=candidate-x
            alpha=1.
            block=-1
            side=0
            for i in np.flatnonzero(free):
                if candidate[i]<lo[i]:
                    step=(lo[i]-x[i])/direction[i]
                    if step<alpha:
                        alpha,block,side=max(0.,step),i,-1
                elif candidate[i]>hi[i]:
                    step=(hi[i]-x[i])/direction[i]
                    if step<alpha:
                        alpha,block,side=max(0.,step),i,1
            x=np.clip(x+alpha*direction,lo,hi)
            if block>=0:
                state[block]=side
                x[block]=lo[block] if side==-1 else hi[block]
                continue
        gradient=H@x+g
        violation=np.where(state==-1,-gradient,np.where(state==1,gradient,0.))
        worst=int(np.argmax(violation))
        if violation[worst]<=tolerance:
            return x
        state[worst]=0
    raise RuntimeError("Bounded least-squares solver did not converge")


def velocity_bounds(qpos, limits, max_velocity, margin, influence, dt):
    """Gradually suppress outward motion near limits; always permit retreat."""
    q=np.asarray(qpos,dtype=float)
    limits=np.asarray(limits,dtype=float)
    if q.shape!=(6,) or limits.shape!=(6,2) or not np.isfinite(q).all() or not np.isfinite(limits).all():
        raise ValueError("Expected six finite joints and joint ranges")
    if not np.isfinite([max_velocity,margin,influence,dt]).all() or min(max_velocity,margin,influence,dt)<=0:
        raise ValueError("Velocity-bound parameters must be positive")
    if np.any(limits[:,1]-limits[:,0]<=2*margin):
        raise ValueError("Joint ranges need room for the margin")
    lower=limits[:,0]+margin
    upper=limits[:,1]-margin
    lo=-max_velocity*np.clip((q-lower)/influence,0.,1.)
    hi=max_velocity*np.clip((upper-q)/influence,0.,1.)
    lo=np.maximum(lo,np.minimum(0.,(lower-q)/dt))
    hi=np.minimum(hi,np.maximum(0.,(upper-q)/dt))
    return lo,hi


def limited_command(sample, J, qpos, limits, config, ibvs_config, dt, alternate=False):
    """Keep the original command when feasible; otherwise solve within bounds."""
    lo,hi=velocity_bounds(qpos,limits,ibvs_config["max_joint_velocity_rad_s"],
        config["joint_limit_margin_rad"],np.deg2rad(config["influence_degrees"]),dt)
    # Uniform scaling in the original controller can exceed a bound by one ULP.
    # Such roundoff must not switch to a different joint-motion solution.
    if not alternate and np.all(sample.velocity>=lo-1e-12) and np.all(sample.velocity<=hi+1e-12):
        return sample.velocity.copy(),False
    damping=ibvs_config["joint_damping"]
    preferred=np.zeros(6)
    if alternate:
        damping=config["retry_joint_damping"]
        center=np.mean(limits,axis=1)
        half=(limits[:,1]-limits[:,0])/2
        preferred=config["centering_velocity_rad_s"]*(center-qpos)/half
    velocity=bounded_least_squares(J,sample.camera_twist,lo,hi,damping,preferred)
    twist=J@velocity
    scale=max(1.,np.linalg.norm(twist[:3])/ibvs_config["max_linear_velocity_m_s"],
              np.linalg.norm(twist[3:])/ibvs_config["max_angular_velocity_rad_s"])
    return velocity/scale,True


class JointLimitSupervisor:
    """Supervise visual alignment; retry once from a genuinely observed view."""
    def __init__(self, limits, config=None):
        from collections import deque
        self.config=dict(load_joint_limit_config() if config is None else config)
        c=self.config
        if type(c["enabled"]) is not bool:
            raise ValueError("enabled must be boolean")
        for key in ("joint_limit_margin_rad","influence_degrees","retry_joint_damping",
                    "centering_velocity_rad_s","stall_window_s","minimum_improvement_px",
                    "minimum_improvement_fraction","stall_min_error_px",
                    "reposition_velocity_rad_s","max_reposition_time_s",
                    "max_reposition_excursion_degrees","reposition_arrival_tolerance_rad",
                    "reposition_position_gain_per_s"):
            if not np.isfinite(c[key]) or c[key]<=0:
                raise ValueError(f"{key} must be positive and finite")
        if type(c["max_retries"]) is not int or c["max_retries"]<0:
            raise ValueError("max_retries must be a nonnegative integer")
        self.limits=np.asarray(limits,dtype=float).copy()
        velocity_bounds(np.mean(self.limits,axis=1),self.limits,.25,c["joint_limit_margin_rad"],.3,1/30)
        self.history=deque()
        self.reset()

    def reset(self):
        self.history.clear()
        self.time_s=0.
        self.anchor=None
        self.retries=0
        self.alternate=False
        self.adjusted_frames=0
        self.stalls=0
        self.events=[]
        self.reposition_active=False
        self.reposition_elapsed_s=0.
        self.confirmed_frames=0

    def clear_progress(self):
        self.history.clear()

    def _event(self,kind,error,qpos,**extra):
        self.events.append(dict(time_s=self.time_s,kind=kind,error_px=error,
                                qpos_rad=np.asarray(qpos).tolist(),**extra))

    def _begin_retry(self,sample,qpos,reason):
        from dataclasses import replace
        zero=np.zeros(6)
        self.stalls+=int(reason=="stalled")
        self._event(reason,sample.error_px,qpos)
        if self.retries>=self.config["max_retries"] or self.anchor is None:
            return replace(sample,status="alignment_stalled" if reason=="stalled" else "timeout",
                           velocity=zero,camera_twist=zero.copy())
        self.retries+=1
        self.reposition_active=True
        self.reposition_elapsed_s=0.
        self.confirmed_frames=0
        self.clear_progress()
        excursion=np.deg2rad(self.config["max_reposition_excursion_degrees"])
        margin=self.config["joint_limit_margin_rad"]
        self.reposition_lower=np.maximum(self.limits[:,0]+margin,qpos-excursion)
        self.reposition_upper=np.minimum(self.limits[:,1]-margin,qpos+excursion)
        self.reposition_goal=np.clip(self.anchor,self.reposition_lower,self.reposition_upper)
        self._event("retry_started",sample.error_px,qpos,reason=reason,
                    goal_qpos_rad=self.reposition_goal.tolist())
        # This first zero command brakes before the joint-feedback return begins.
        return replace(sample,status="repositioning",velocity=zero,camera_twist=zero.copy())

    def filter(self,sample,J,qpos,dt,ibvs_config,clock_s=None,observation_qpos=None):
        from dataclasses import replace
        if not self.config["enabled"]:
            return sample
        self.time_s=self.time_s+dt if clock_s is None else clock_s
        if sample.status=="timeout":
            return self._begin_retry(sample,qpos,"alignment_timeout")
        if sample.status!="running":
            self.clear_progress()
            return sample
        if self.anchor is None:
            self.anchor=np.asarray(qpos if observation_qpos is None else observation_qpos,dtype=float).copy()
        if not np.any(sample.velocity):
            self.clear_progress()
            return sample
        try:
            velocity,adjusted=limited_command(sample,J,qpos,self.limits,self.config,ibvs_config,dt,self.alternate)
        except (np.linalg.LinAlgError,RuntimeError):
            self._event("solver_failure",sample.error_px,qpos)
            return replace(sample,status="solver_failure",velocity=np.zeros(6),camera_twist=np.zeros(6))
        self.adjusted_frames+=int(adjusted)
        error=sample.error_px
        if error>self.config["stall_min_error_px"]:
            self.history.append((self.time_s,error))
            window=self.config["stall_window_s"]
            while len(self.history)>1 and self.history[1][0]<=self.time_s-window:
                self.history.popleft()
            if self.time_s-self.history[0][0]+1e-9>=window:
                midpoint=self.time_s-window/2
                before=[e for t,e in self.history if t<=midpoint]
                after=[e for t,e in self.history if t>midpoint]
                if before and after:
                    required=max(self.config["minimum_improvement_px"],
                                 self.config["minimum_improvement_fraction"]*min(before))
                    if min(before)-min(after)<required:
                        return self._begin_retry(sample,qpos,"stalled")
        else:
            self.clear_progress()
        if adjusted:
            return replace(sample,status="realigning" if self.alternate else "joint_limited",
                           velocity=velocity,camera_twist=J@velocity)
        return sample

    def reposition(self,corners,qpos,dt,confirmation_frames,clock_s=None):
        from control import ControlSample
        zero=np.zeros(6)
        def sample(status,velocity=None):
            return ControlSample(status,None,zero.copy() if velocity is None else velocity,zero.copy())
        self.time_s=self.time_s+dt if clock_s is None else clock_s
        if not self.reposition_active:
            return sample("reposition_timeout")
        if self.reposition_elapsed_s+1e-9>=self.config["max_reposition_time_s"]:
            self.reposition_active=False
            self._event("reposition_timeout",None,qpos)
            return sample("reposition_timeout")
        self.reposition_elapsed_s+=dt
        delta=self.reposition_goal-qpos
        if np.max(np.abs(delta))>self.config["reposition_arrival_tolerance_rad"]:
            self.confirmed_frames=0
            velocity=self.config["reposition_position_gain_per_s"]*delta
            velocity*=min(1.,self.config["reposition_velocity_rad_s"]/max(float(np.max(np.abs(velocity))),1e-12))
            velocity=np.clip(velocity,np.minimum(0.,(self.reposition_lower-qpos)/dt),
                             np.maximum(0.,(self.reposition_upper-qpos)/dt))
            return sample("repositioning",velocity)
        valid=(corners is not None and np.shape(corners)==(4,2) and np.isfinite(corners).all())
        self.confirmed_frames=self.confirmed_frames+1 if valid else 0
        if self.confirmed_frames<confirmation_frames:
            return sample("retry_confirming")
        self.reposition_active=False
        self.alternate=True
        self.clear_progress()
        self._event("retry_confirmed",None,qpos)
        return sample("retry_ready")
