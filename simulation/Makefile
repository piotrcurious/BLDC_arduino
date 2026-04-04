CXX = g++
CXXFLAGS = -I. -lpthread
SRCS = Arduino.cpp
SKETCHES = test_speed_control test_max_current test_mppt test_extra_regen test_scenarios test_full_suite test_mppt_perf test_robustness test_param_accuracy test_faults test_stress_dynamic test_soak test_step_response test_load_disturbance

all: $(SKETCHES)

%: %.cpp $(SRCS)
	$(CXX) -I. $^ -o $@ $(CXXFLAGS)

clean:
	rm -f $(SKETCHES)

.PHONY: all clean
