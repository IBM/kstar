from contextlib import contextmanager
from typing import Any
import os
import tempfile as tmp


@contextmanager
def temp_file(suffix: str, dir_name: Any = None) -> Any:
    tf = tmp.NamedTemporaryFile(delete=False, suffix=suffix, dir=dir_name)
    tf.file.close()
    try:
        yield tf.name
    finally:
        try:
            os.remove(tf.name)
        except OSError as e:
            if e.errno == 2:
                pass
            else:
                raise


@contextmanager
def open_atomic(filepath: Any, *args: Any, **kwargs: Any) -> Any:
    fsync = kwargs.pop("fsync", False)

    with temp_file(suffix="", dir_name=os.path.dirname(os.path.abspath(filepath))) as temp_path:
        with open(temp_path, *args, **kwargs) as file:
            try:
                yield file
            finally:
                if fsync:
                    file.flush()
                    os.fsync(file.fileno())
        os.rename(temp_path, filepath)
