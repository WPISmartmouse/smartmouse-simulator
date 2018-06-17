#!/bin/bash

cd .sim_build_asan && make && make test && cd -
cd .sim_build_lsan && make && make test && cd -
cd .sim_build_tsan && make && make test && cd -
cd .sim_build_ubsan && make && make test && cd -
