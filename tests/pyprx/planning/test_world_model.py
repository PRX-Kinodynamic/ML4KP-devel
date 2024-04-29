import os
import math
import pytest
import PyML4KP as prx

def empty_world_model_test():
  world_model = prx.world_model_t();

  world_model.create_context("test_context", [], []);

  context = world_model.get_context("test_context");
  assert(world_model.get_all_context_names()[0] == "test_context")
