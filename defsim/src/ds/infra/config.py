#!/usr/bin/env python3

from functools import wraps, partial
from typing import (
    Callable, Union, Type,
    TypeVar, Optional
)
from dataclasses import is_dataclass
from omegaconf import OmegaConf
import inspect

from hydra.conf import HydraConf
from hydra_zen import (store, zen)
D = TypeVar('D')


def zen_cli(f=None, *args, **kwds):
    """
    Thin hydra-zen wrapper.
    Useful when trying to get help-text regarding CLI args.
    """
    if f is None:
        return partial(zen_cli, *args, **kwds)

    config_cls = None
    sig = inspect.signature(f)
    if len(sig.parameters) == 1:
        key = next(iter(sig.parameters))
        config_cls = sig.parameters[key].annotation
        if not is_dataclass(config_cls):
            config_cls = None

    if config_cls is not None:
        f0 = f

        @wraps(config_cls)
        def f(*args, **kwds):
            return f0(config_cls(*args, **kwds))

    # disable generating `outputs/` subdir
    f = store(f, name='main',
              hydra_defaults=["_self_",
                              {"override hydra/hydra_logging": "disabled"},
                              {"override hydra/job_logging": "disabled"},
                              ],
              )
    c = HydraConf(output_subdir=None)
    c.run.dir = '.'
    _ = store(c)

    @wraps(f)
    def wrapper(*args, **kwds):
        store.add_to_hydra_store()
        return zen(f).hydra_main(
            config_name='main',
            version_base='1.2',
            config_path=None)

    return wrapper


def oc_cli(cls: Union[Type[D], D] = None,
           cfg_file: Optional[str] = None):
    """
    Decorator for automatically adding parsed args from cli to entry point.
    Useful when dealing with nested configs.
    """

    main = None
    if cls is None:
        # @with_cli()
        need_cls = True
    else:
        if callable(cls) and not is_dataclass(cls):
            # @with_cli
            main = cls
            need_cls = True
        else:
            # @with_cli(cls=Config, ...)
            need_cls = (cls is None)  # FIXME(ycho): always False.

    def decorator(main: Callable[[D], None]):
        # NOTE(ycho):
        # if `cls` is None, try to infer them from `main` signature.
        inner_cls = cls
        if need_cls:
            sig = inspect.signature(main)
            if len(sig.parameters) == 1:
                key = next(iter(sig.parameters))
                inner_cls = sig.parameters[key].annotation
            else:
                raise ValueError(
                    '#arg != 1 in main {}: Cannot infer param type.'
                    .format(sig))

        # NOTE(ycho): using @wraps to forward main() documentation.
        @wraps(main)
        def wrapper():
            getattr(main, '__doc__', '')
            cfg = OmegaConf.structured(inner_cls)
            if cfg_file is not None:
                cfg.merge_with(OmegaConf.load(cfg_file))
            cfg.merge_with_cli()
            cfg = OmegaConf.to_object(cfg)
            return main(cfg)
        return wrapper

    if main is not None:
        return decorator(main)
    else:
        return decorator
