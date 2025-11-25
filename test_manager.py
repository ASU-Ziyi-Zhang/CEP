#! /usr/bin/env python3
################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

import unittest

def run_tests():
    """Runs all test suites."""
    loader = unittest.TestLoader()
    test_suite = unittest.TestSuite()

    # Discover and add all tests in the tests/ directory
    test_suite.addTests(loader.discover("tests"))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)

    # Summary Report
    print("\n======== TEST SUMMARY ========")
    print(f"Total tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.wasSuccessful():
        print("✅ All tests passed successfully!")
    else:
        print("⚠️ Some tests failed or had errors. Check the logs.")

if __name__ == "__main__":
    run_tests()