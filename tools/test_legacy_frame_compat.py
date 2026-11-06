"""Exercise the official decoders without importing their GUI dependencies.

This tests payload compatibility, not adapter transport or GUI operation.
Run: python3 -m unittest discover -s tools -p test_legacy_frame_compat.py
"""
import ast
import math
from pathlib import Path
import random
import struct
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parent


class LegacyFrameCompatibility(unittest.TestCase):
    def test_official_ahrs_decoders_ignore_extensions(self):
        constants = {"GRAVITY", "ACCEL_LSB_PER_G", "GYRO_LSB_PER_DPS", "DEG"}
        for filename in ("ahrs_demo.py", "ahrs_lib_compare.py"):
            tree = ast.parse((ROOT / filename).read_text())
            nodes = [n for n in tree.body if
                     (isinstance(n, ast.FunctionDef) and n.name in ("_s16", "decode_frame")) or
                     (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in constants for t in n.targets))]
            namespace = {"math": math, "np": SimpleNamespace(array=list)}
            exec(compile(ast.Module(body=nodes, type_ignores=[]), filename, "exec"), namespace)
            rng = random.Random(431)
            for _ in range(1000):
                original = struct.pack(">H7h", rng.randrange(65536), *(rng.randrange(-32768, 32768) for _ in range(7)))
                expected = namespace["decode_frame"](original)
                for length in (24, 32):
                    with self.subTest(decoder=filename, length=length):
                        self.assertEqual(namespace["decode_frame"](original + rng.randbytes(length - 16)), expected)

    def test_official_eskf_decoder_ignores_extensions(self):
        tree = ast.parse((ROOT / "eskf.py").read_text())
        call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Attribute) and isinstance(n.func.value, ast.Name)
                    and n.func.value.id == "struct" and n.func.attr == "unpack")
        expression = compile(ast.Expression(body=call), "eskf.py", "eval")
        rng = random.Random(45686)
        for _ in range(1000):
            values = (rng.randrange(65536), *(rng.randrange(-32768, 32768) for _ in range(7)))
            original = struct.pack(">H7h", *values)
            for length in (16, 24, 32):
                result = eval(expression, {"struct": struct, "msg": SimpleNamespace(data=original + rng.randbytes(length - 16))})
                self.assertEqual(result, values)


if __name__ == "__main__":
    unittest.main()
