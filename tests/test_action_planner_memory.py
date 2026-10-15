import logging
import sys
import unittest
from pathlib import Path

from unified_planning.io import PDDLReader
from unified_planning.shortcuts import And, Equals, FluentExp, Not, Or


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/src/drone_planner/scripts"))
from action_planner_memory import ActionPlannerMemory


class ConditionEvaluationTests(unittest.TestCase):
    def setUp(self):
        domain = ROOT / "src/src/drone_planner/pddl/harpia_domain.pddl"
        self.memory = ActionPlannerMemory()
        self.memory.pddl = PDDLReader().parse_problem(str(domain))
        self.memory.logger = logging.getLogger(__name__)
        self.memory.add_instance("base", "base_1")
        self.memory.add_instance("region", "region_1")

    def test_go_to_checks_negated_object_equality(self):
        self.memory.add_predicate("at", ["base_1"])
        self.memory.add_predicate("is_flying", [])
        self.memory.set_function("battery_capacity", [], 100)

        self.assertTrue(self.memory.check_conditions_action(
            "go_to", ["base_1", "region_1"], ["at start", "over all"]))
        self.assertFalse(self.memory.check_conditions_action(
            "go_to", ["base_1", "base_1"], ["at start", "over all"]))

    def test_negated_predicate_and_nested_logical_conditions(self):
        pddl = self.memory.pddl
        base = pddl.object("base_1")
        region = pddl.object("region_1")
        at_base = FluentExp(pddl.fluent("at"), [base])
        at_region = FluentExp(pddl.fluent("at"), [region])
        self.memory.add_predicate("at", ["base_1"])

        self.assertTrue(self.memory._are_conditions_met([
            And(at_base, Or(Not(at_region), Equals(base, region)))
        ]))
        self.assertFalse(self.memory._are_conditions_met([Not(at_base)]))


if __name__ == "__main__":
    unittest.main()
