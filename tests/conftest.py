import sys
import os
import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(TEST_DIR)
ROOT_DIR = os.path.dirname(TEST_DIR)
sys.path.append(ROOT_DIR)

def run_this_test(file):
    pytest.main([os.path.abspath(file)])