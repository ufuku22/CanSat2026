"""NavigationControllerのデフォルト値をテストから参照するための補助関数。"""

from __future__ import annotations

from inspect import Parameter, signature
from typing import Any, Callable


def navigation_default(method: Callable[..., Any], parameter_name: str) -> Any:
    """NavigationController側に定義された引数のデフォルト値を返す。"""
    parameter = signature(method).parameters[parameter_name]
    if parameter.default is Parameter.empty:
        raise ValueError(f"{method.__name__}.{parameter_name} has no default")
    return parameter.default
