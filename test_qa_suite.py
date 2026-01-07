#!/usr/bin/env python3
"""
Test script to run the QA test suite.
"""
import os
import sys

# Add the project root and backend to the Python path
project_root = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, 'backend'))

# Now import and run the QA test suite
from backend.qa_test_suite import run_qa_tests

def test_qa_suite():
    print("Running QA Test Suite...")
    try:
        # Run the test suite (this will use mock data since we don't have real API keys)
        test_results = run_qa_tests()
        print(f"QA Test Suite completed. Overall accuracy: {test_results.overall_accuracy:.2%}")

        if test_results.overall_accuracy >= 0.90:
            print("SUCCESS: QA Test Suite passed with 90%+ accuracy!")
            return True
        else:
            print(f"FAILURE: QA Test Suite only achieved {test_results.overall_accuracy:.2%} accuracy (target: 90%+)")
            return False
    except Exception as e:
        print(f"ERROR running QA Test Suite: {str(e)}")
        return False

if __name__ == "__main__":
    success = test_qa_suite()
    if success:
        print("\n[SUCCESS] QA Test Suite validation passed")
    else:
        print("\n[ERROR] QA Test Suite validation failed")
        sys.exit(1)