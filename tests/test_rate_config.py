import os
import unittest

from fullstack_manip.utils.rate_config import RateConfig, load_rate_config


class RateConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self._original_env = {
            key: value
            for key, value in os.environ.items()
            if key.startswith("RATES__")
        }
        for key in list(os.environ.keys()):
            if key.startswith("RATES__"):
                del os.environ[key]

    def tearDown(self) -> None:
        for key in list(os.environ.keys()):
            if key.startswith("RATES__"):
                del os.environ[key]
        os.environ.update(self._original_env)

    def test_load_rate_config_defaults(self) -> None:
        rates = load_rate_config(defaults=RateConfig())
        self.assertGreater(rates.sim, 0.0)
        self.assertGreater(rates.control, 0.0)

    def test_env_override(self) -> None:
        os.environ["RATES__CONTROL"] = "0.005"
        rates = load_rate_config(defaults=RateConfig())
        self.assertAlmostEqual(rates.control, 0.005, places=9)

    def test_frequency_override(self) -> None:
        os.environ["RATES__PLANNER"] = "2Hz"
        rates = load_rate_config(defaults=RateConfig())
        self.assertAlmostEqual(rates.planner, 0.5, places=9)


if __name__ == "__main__":
    unittest.main()
