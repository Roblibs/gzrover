#!/bin/bash

#run in pause mode
gzserver -u my.world --profile top_ode --verbose

gzserver -u my.world --profile higher_ode --verbose

gzserver my.world --profile bullet --verbose -e bullet

gazebo my.world --profile top_ode --verbose -r --record_path ~/log

gazebo -u --verbose -p ~/log/state.log
