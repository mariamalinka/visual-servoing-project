"""Bounded, fixed-shape graph replay for the pinned LightGlue transformer blocks."""
from __future__ import annotations


class BlockGraph:
    """Single-owner replay; returned GPU buffers are borrowed until the next call.

    The perception worker completes its existing D2H result copy before the next
    image. Never share a detector across concurrent callers or retain block
    outputs as a cross-frame feature cache. Shapes outside the one captured
    signature run eagerly, without creating additional graphs or queues.
    """
    def __init__(self,torch,module,args,stream):
        self.torch=torch
        self.module=module
        self.forward=module.forward
        self.replays=0
        self.fallbacks=0
        self.signature=tuple((a.shape,a.dtype,a.device) for a in args)
        with torch.inference_mode(),torch.cuda.stream(stream):
            self.inputs=tuple(a.clone() for a in args)
            # Create the stream's BLAS handle before capture. Capture records the
            # block; the existing camera warm-up performs its first execution.
            torch.cuda.current_blas_handle()
            self.graph=torch.cuda.CUDAGraph()
            with torch.cuda.graph(self.graph,stream=stream):
                self.outputs=self.forward(*self.inputs)

    def __call__(self,*args,mask0=None,mask1=None):
        if (mask0 is not None or mask1 is not None or self.module.training or
            not self.torch.is_inference_mode_enabled() or
            tuple((a.shape,a.dtype,a.device) for a in args)!=self.signature):
            self.fallbacks+=1
            return self.forward(*args,mask0=mask0,mask1=mask1)
        for target,source in zip(self.inputs,args):target.copy_(source)
        self.graph.replay()
        self.replays+=1
        return self.outputs


def install_block_graphs(detector):
    torch=detector.torch
    matcher=detector.matcher
    if detector.device.type!='cuda':return []
    stream=torch.cuda.Stream()
    stream.wait_stream(torch.cuda.current_stream())
    with torch.inference_mode(),torch.cuda.stream(stream):
        m=detector.template_features['keypoints'].shape[1]
        n=detector.config['max_keypoints']
        d=matcher.conf.descriptor_dim
        a=torch.zeros((1,m,d),device=detector.device)
        b=torch.zeros((1,n,d),device=detector.device)
        ea=matcher.posenc(torch.zeros((1,m,2),device=detector.device))
        eb=matcher.posenc(torch.zeros((1,n,2),device=detector.device))
        graphs=[BlockGraph(torch,block,(a,b,ea,eb),stream) for block in matcher.transformers]
    torch.cuda.current_stream().wait_stream(stream)
    for block,graph in zip(matcher.transformers,graphs):block.forward=graph
    return graphs
