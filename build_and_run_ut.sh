#!/bin/bash

# root directory
PROJECT_ROOT=`pwd`

CMAKE_FLAGS=$@

function run_unit_test {
	mkdir -p $1/b
	pushd $1/b > /dev/null

	cmake -DENABLE_COVERAGE=1 -DENABLE_TESTS=1 $CMAKE_FLAGS ..
	if [ $? -ne 0 ]; then
		echo "Errors generating makefiles"
		exit 1
	fi

	make
	if [ $? -ne 0 ]; then
		echo "Errors building unittest"
		exit 1
	fi

	make coverage
	if [ $? -ne 0 ]; then
		echo "Errors creating coverage report"
		exit 1
	fi

	make coverage-html
	if [ $? -ne 0 ]; then
		echo "Errors creating HTML coverage report"
		exit 1
	fi

	popd > /dev/null
}

for COMPONENT in common calibration filter ; do
	if [ -d $(pwd)/$COMPONENT/test/ut ]; then
		echo "*** Running unit tests for $COMPONENT"
		run_unit_test $(pwd)/$COMPONENT/test/ut
	else
		echo "Unable to find unit tests for component $COMPONENT in $(pwd)/${COMPONENT}/test/ut"
		exit 1
	fi
	echo ""
done