"""Run desktop checks against the sensor's actual C++ fuzzy module.

Run: python3 -B -m unittest discover -s tests -p 'test_fuzzy_*.py' -v
These checks do not validate sensor readings or fire detection performance.
"""

import itertools
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class LowHeatFuzzyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("clang++") or shutil.which("c++")
        if compiler is None:
            raise unittest.SkipTest("A C++ compiler is required.")
        cls.directory = tempfile.TemporaryDirectory(prefix="beforefire-fuzzy-")
        cls.addClassCleanup(cls.directory.cleanup)
        directory = Path(cls.directory.name)
        sensor = Path(__file__).resolve().parents[1] / "Sensor"
        runner = directory / "runner.cpp"
        runner.write_text(r'''
#include "FuzzyKompor.h"
#include <iomanip>
#include <iostream>
int main() {
    FuzzyInput input{};
    int present;
    std::cout << std::setprecision(10);
    while (std::cin >> input.stoveTemperatureC >> input.iaq >> present
           >> input.absenceDurationS >> input.roomTemperatureC >> input.roomHumidityRH) {
        input.presence = present != 0;
        const auto result = evaluateFuzzy(input);
        std::cout << result.valid << ' ' << static_cast<int>(result.condition)
                  << ' ' << result.score << ' ' << static_cast<int>(result.ruleId) << '\n';
    }
}
''')
        cls.binary = directory / "runner"
        subprocess.run(
            [compiler, "-std=c++17", "-O2", "-I", str(sensor), str(runner),
             str(sensor / "FuzzyKompor.cpp"), "-o", str(cls.binary)],
            check=True, capture_output=True, text=True,
        )

    @classmethod
    def evaluate(cls, cases):
        # A case is (stove temperature, IAQ, presence, absence seconds).
        request = "".join(f"{t} {q} {int(p)} {d} 28 55\n" for t, q, p, d in cases)
        output = subprocess.run([str(cls.binary)], input=request, capture_output=True,
                                text=True, check=True)
        results = []
        for line in output.stdout.splitlines():
            valid, condition, score, rule = line.split()
            results.append((int(valid), int(condition), float(score), int(rule)))
        if len(results) != len(cases):
            raise AssertionError("The C++ runner did not return every result.")
        return results

    def test_low_heat_is_normal_for_all_iaq_categories_and_absence_lengths(self):
        temperatures = sorted(set(range(0, 151, 5)) | {32, 149})
        contexts = [(True, 0)] + [(False, d) for d in (0, 1, 150, 180, 210, 300, 420, 600, 900, 3600)]
        cases = [(t, q, p, d) for t, q, (p, d) in
                 itertools.product(temperatures, range(0, 501, 5), contexts)]
        for case, result in zip(cases, self.evaluate(cases)):
            self.assertEqual(result[:2], (1, 0), case)
            self.assertLess(result[2], 25, case)
            self.assertIn(result[3], range(1, 8), case)

    def test_presence_change_at_32_degrees_does_not_raise_the_label(self):
        for iaq in (225, 300, 350, 355, 425, 500):
            detected, absent = self.evaluate([(32, iaq, True, 0), (32, iaq, False, 1)])
            self.assertEqual(detected[:3], absent[:3])
            self.assertEqual(absent[:2], (1, 0))

    def test_revised_rule_ids_remain_available(self):
        cases = [(32, 125, False, 900), (32, 175, False, 360), (32, 425, False, 1)]
        for result, rule in zip(self.evaluate(cases), (3, 5, 7)):
            self.assertEqual(result[:2], (1, 0))
            self.assertEqual(result[3], rule)

    def test_transition_from_150_to_160_has_no_missing_output(self):
        cases = [(150 + i / 4, q, p, d) for i, q, (p, d) in itertools.product(
            range(41), (75, 125, 150, 175, 200, 300, 350, 425, 500),
            ((True, 0), (False, 0), (False, 240), (False, 900)))]
        for case, result in zip(cases, self.evaluate(cases)):
            self.assertEqual(result[0], 1, case)
            self.assertGreaterEqual(result[2], 0, case)
            self.assertLessEqual(result[2], 100, case)

    def test_higher_temperature_warning_and_alarm_examples_are_preserved(self):
        cases = [
            ((180, 75, False, 240), 1, 9),
            ((180, 425, True, 0), 1, 16),
            ((180, 425, False, 0), 2, 17),
            ((230, 75, False, 360), 2, 20),
            ((230, 425, False, 0), 2, 25),
            ((260, 175, False, 0), 3, 31),
            ((260, 425, True, 0), 3, 32),
            ((260, 425, False, 0), 3, 32),
        ]
        results = self.evaluate([case for case, _, _ in cases])
        for (case, condition, rule), result in zip(cases, results):
            self.assertEqual(result[:2], (1, condition), case)
            self.assertEqual(result[3], rule, case)


if __name__ == "__main__":
    unittest.main()
