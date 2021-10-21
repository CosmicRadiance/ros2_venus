#!/bin/bash
#
# This script runs docker build to create a docker image.
#

set -exu   # http://linuxcommand.org/lc3_man_pages/seth.html

tag_name=ros2:venus

docker build -f Dockerfile --tag ${tag_name} .
