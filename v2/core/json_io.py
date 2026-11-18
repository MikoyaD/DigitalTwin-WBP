"""Lossless JSON boundary for V2 numeric results, with exclusive atomic publication."""
import json
import math
import os
import tempfile
from pathlib import Path
import numpy as np


def compatible(value, location='$'):
    """Preserve every array element and binary64 scalar; reject unsupported data."""
    if isinstance(value, np.ndarray):
        return compatible(value.tolist(), location)
    if isinstance(value, np.generic):
        converted=value.item()
        if isinstance(converted, np.generic):
            raise TypeError('%s: unsupported NumPy precision/type %s' % (location,value.dtype))
        return compatible(converted, location)
    if value is None or isinstance(value,(str,bool,int)):
        return value
    if isinstance(value,float):
        if not math.isfinite(value):raise ValueError('%s: nonfinite JSON number' % location)
        return value
    if isinstance(value,dict):
        if any(not isinstance(key,str) for key in value):
            raise TypeError('%s: JSON object keys must be strings' % location)
        return {key:compatible(item,location+'.'+key) for key,item in value.items()}
    if isinstance(value,(list,tuple)):
        return [compatible(item,'%s[%d]'%(location,i)) for i,item in enumerate(value)]
    raise TypeError('%s: unsupported JSON value %s' % (location,type(value).__name__))


def write_exclusive(path,value):
    """Serialize before touching the destination; a reader never sees a partial file.

    Same-directory temporary + atomic exclusive link preserves an existing target,
    including in races. Any serialization/I/O failure removes the temporary file.
    """
    path=Path(path)
    text=json.dumps(compatible(value),ensure_ascii=False,indent=2,allow_nan=False)
    temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=str(path.parent),
                                         prefix='.'+path.name+'.',delete=False) as stream:
            temporary=Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(str(temporary),str(path))
    finally:
        if temporary is not None:temporary.unlink(missing_ok=True)
