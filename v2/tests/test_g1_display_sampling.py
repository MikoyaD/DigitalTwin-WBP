import numpy as np
import pytest
from v2.scripts.g1_display_sampling import select_rows

def test_events_extrema_and_raw_rows():
    a=np.zeros((1001,41));a[:,0]=np.arange(1001)*.001
    a[:,10]=np.sin(a[:,0]*8);a[:,19]=-a[:,10];a[:,28]=np.cos(a[:,0]*19)
    a[500:,39]=1;a[500:,40]=1;a[500:,29]=1
    ix=select_rows(a,[0,.5,1]);z=a[ix]
    assert np.all(np.diff(z[:,0])<=.010000000001)
    assert {0,499,500,501,1000}.issubset(set(ix))
    for col in range(10,37):
        assert z[:,col].min()==a[:,col].min() and z[:,col].max()==a[:,col].max()
    assert np.array_equal(z,a[ix])

def test_absent_event_and_source_gap_preserved():
    a=np.zeros((3,41));a[:,0]=[0,.02,.04]
    with pytest.raises(ValueError,match='EVENT_NOT_PRESENT'):select_rows(a,[.005])
    assert select_rows(a,[0,.04]).tolist()==[0,1,2]

def test_mixed_source_gaps_only_unavoidable_gaps_exceed_limit():
    a=np.zeros((8,41));a[:,0]=[0,.004,.009,.025,.029,.033,.037,.042]
    ix=select_rows(a,[0,.025,.042]); assert {2,3}.issubset(ix)
    for i,j in zip(ix[:-1],ix[1:]):
        assert a[j,0]-a[i,0]<=.010000000001 or (j==i+1 and a[j,0]-a[i,0]>.01)

def test_display_status_distinguishes_dense_acceptance():
    from v2.adapters.run_discovery import preview_status
    result={'status':'INDEPENDENT_NUMERICAL_PASS','display_sampling':{'dense_rows':100}}
    assert preview_status(result,{'status':'PASS'})=='DENSE_SOURCE_VERIFIED_DISPLAY_ONLY'
    assert preview_status(result,{})=='COLLISION_PENDING'
    assert preview_status({'status':'INDEPENDENT_NUMERICAL_PASS'},{'status':'PASS'})=='VERIFIED'

    result['collision']='NOT_ATTEMPTED'
    assert preview_status(result,{})=='COLLISION_NOT_ATTEMPTED'
