#!/usr/bin/env python3
"""
Verification script for behavior tree implementation.

This script verifies that all behaviors are properly implemented
and can be imported without errors.
"""

import sys


def verify_imports():
    """Verify all behavior imports."""
    print("=" * 60)
    print("Behavior Tree Implementation Verification")
    print("=" * 60)
    print()

    # Test core imports
    print("✓ Testing core imports...")
    try:
        from fullstack_manip.execution import Blackboard, BaseBehavior, Status

        print("  ✓ Blackboard imported")
        print("  ✓ BaseBehavior imported")
        print("  ✓ Status imported")
    except ImportError as e:
        print(f"  ✗ Core import failed: {e}")
        return False

    # Test motion behaviors
    print("\n✓ Testing motion behaviors...")
    try:
        from fullstack_manip.execution import (
            MoveToJointConfiguration,
            MoveToCartesianPose,
            ExecuteTrajectory,
        )

        print("  ✓ MoveToJointConfiguration imported")
        print("  ✓ MoveToCartesianPose imported")
        print("  ✓ ExecuteTrajectory imported")
    except ImportError as e:
        print(f"  ✗ Motion behaviors import failed: {e}")
        return False

    # Test gripper behaviors
    print("\n✓ Testing gripper behaviors...")
    try:
        from fullstack_manip.execution import (
            OpenGripper,
            CloseGripper,
            VerifyGrasp,
            SetGripperPosition,
        )

        print("  ✓ OpenGripper imported")
        print("  ✓ CloseGripper imported")
        print("  ✓ VerifyGrasp imported")
        print("  ✓ SetGripperPosition imported")
    except ImportError as e:
        print(f"  ✗ Gripper behaviors import failed: {e}")
        return False

    # Test perception behaviors
    print("\n✓ Testing perception behaviors...")
    try:
        from fullstack_manip.execution import (
            DetectObject,
            EstimateObjectPose,
            CheckDetectionStatus,
            VisuallyAlign,
            UpdateSceneState,
        )

        print("  ✓ DetectObject imported")
        print("  ✓ EstimateObjectPose imported")
        print("  ✓ CheckDetectionStatus imported")
        print("  ✓ VisuallyAlign imported")
        print("  ✓ UpdateSceneState imported")
    except ImportError as e:
        print(f"  ✗ Perception behaviors import failed: {e}")
        return False

    # Test safety behaviors
    print("\n✓ Testing safety behaviors...")
    try:
        from fullstack_manip.execution import (
            MonitorJointLimits,
            MonitorWorkspaceBounds,
            CheckForceLimit,
            EmergencyStop,
            CheckCollision,
            ValidateTrajectory,
        )

        print("  ✓ MonitorJointLimits imported")
        print("  ✓ MonitorWorkspaceBounds imported")
        print("  ✓ CheckForceLimit imported")
        print("  ✓ EmergencyStop imported")
        print("  ✓ CheckCollision imported")
        print("  ✓ ValidateTrajectory imported")
    except ImportError as e:
        print(f"  ✗ Safety behaviors import failed: {e}")
        return False

    # Test control behaviors
    print("\n✓ Testing control behaviors...")
    try:
        from fullstack_manip.execution import (
            ImpedanceControlBehavior,
            AdmittanceControlBehavior,
            TrajectoryFollowingBehavior,
            ForceControlBehavior,
            GravityCompensationBehavior,
            PickPlaceControlBehavior,
            VisualServoControlBehavior,
        )

        print("  ✓ ImpedanceControlBehavior imported")
        print("  ✓ AdmittanceControlBehavior imported")
        print("  ✓ TrajectoryFollowingBehavior imported")
        print("  ✓ ForceControlBehavior imported")
        print("  ✓ GravityCompensationBehavior imported")
        print("  ✓ PickPlaceControlBehavior imported")
        print("  ✓ VisualServoControlBehavior imported")
    except ImportError as e:
        print(f"  ✗ Control behaviors import failed: {e}")
        return False

    return True


def test_basic_functionality():
    """Test basic functionality of core classes."""
    print("\n" + "=" * 60)
    print("Testing Basic Functionality")
    print("=" * 60)
    print()

    try:
        from fullstack_manip.execution import Blackboard, Status

        # Test Blackboard
        print("✓ Testing Blackboard...")
        bb = Blackboard()
        bb.set("test_key", "test_value")
        assert bb.get("test_key") == "test_value"
        assert bb.exists("test_key")
        bb.delete("test_key")
        assert not bb.exists("test_key")
        print("  ✓ Blackboard set/get/exists/delete working")

        # Test Status enum
        print("\n✓ Testing Status enum...")
        assert Status.SUCCESS.value == "SUCCESS"
        assert Status.FAILURE.value == "FAILURE"
        assert Status.RUNNING.value == "RUNNING"
        assert Status.INVALID.value == "INVALID"
        print("  ✓ Status enum values correct")

        return True

    except Exception as e:
        print(f"  ✗ Functionality test failed: {e}")
        return False


def check_py_trees():
    """Check if py-trees is installed."""
    print("\n" + "=" * 60)
    print("Checking py-trees Installation")
    print("=" * 60)
    print()

    try:
        import py_trees

        try:
            version = py_trees.__version__
        except AttributeError:
            # Some versions don't have __version__, try getting it from metadata
            try:
                from importlib.metadata import version as get_version

                version = get_version("py-trees")
            except Exception:
                version = "unknown"
        print(f"✓ py-trees version {version} installed")
        return True
    except ImportError:
        print("✗ py-trees not installed")
        print("  Install with: pip install py-trees>=2.2.0")
        return False


def main():
    """Run all verification tests."""
    results = []

    # Check py-trees
    results.append(("py-trees", check_py_trees()))

    # Verify imports
    results.append(("Imports", verify_imports()))

    # Test functionality
    results.append(("Functionality", test_basic_functionality()))

    # Print summary
    print("\n" + "=" * 60)
    print("Verification Summary")
    print("=" * 60)
    print()

    all_passed = True
    for test_name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{status}: {test_name}")
        if not passed:
            all_passed = False

    print()
    if all_passed:
        print("🎉 All verifications passed!")
        print()
        print("Next steps:")
        print("1. Create unit tests for behaviors")
        print("2. Implement composite skills")
        print("3. Build complete task trees")
        print("4. Test in simulation")
        return 0
    else:
        print("⚠️  Some verifications failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
