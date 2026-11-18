"""Version-4 TM feedback frozen geometry, independent of legacy decoders."""
from pathlib import Path
from hashlib import sha256
from . import tm_frozen
from ..core.contracts import require

VARIANT='V2-TM-RHO-FEEDBACK-v1'

def freeze(block,path,decision,parent,limits):
    item=tm_frozen.freeze(block,path,decision,parent,limits)
    item['contract_version']=4;item['algorithm_variant']=VARIANT
    root=Path(__file__).resolve().parents[1]
    for name in ('geometry/tm_rho_feedback.py','geometry/tm_rho_feedback_joint.py','geometry/tm_rho_feedback_prepare.py','geometry/tm_rho_feedback_loop.py','geometry/tm_rho_feedback_context.py','geometry/tm_rho_feedback_build.py','geometry/tm_rho_feedback_frozen.py','scripts/tm_feedback_hg.py'):
        item['source_identity'][name]=sha256((root/name).read_bytes()).hexdigest()
    return item

def thaw(item):
    if item.get('algorithm_variant')=='V2-SIGNED-H-v1':
        from .signed_frozen import thaw_signed
        return thaw_signed(item)
    require(item.get('contract_version')==4 and item.get('algorithm_variant')==VARIANT,'TM_FEEDBACK_CONTRACT','v4 required')
    old=dict(item);old['contract_version']=3;old['algorithm_variant']='V2-TM-HG'
    return tm_frozen.thaw(old)
