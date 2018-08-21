#!/bin/bash

set -o errexit

source /opt/ros/${ROS_DISTRO}/setup.bash

set -o verbose

cd /catkin_ws

apt-get -qq update
apt-get install libxml2-utils
rosdep install --from-paths src/${PACKAGE_NAME} --ignore-src --rosdistro=${ROS_DISTRO} -y
apt-get clean
rm -rf /var/lib/apt/lists/*

sed -i -e '5a set(CMAKE_C_FLAGS "-Wall -Werror")' \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake
sed -i -e '5a set(CMAKE_CXX_FLAGS "-Wall -Werror")' \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake

CM_OPTIONS=""
LOG=/tmp/catkin_make.log
function error_log() {
  grep -A5 error ${LOG} > ${LOG}.error
  echo -e '\n```'
  head -n30 ${LOG}.error
  ERR_LINES=`cat ${LOG}.error | wc -l`
  if [ $ERR_LINES -gt 30 ]
  then
    echo '--'
    echo "error log exceeded 30 lines (total $ERR_LINES lines)"
  fi
  echo -e '```\n'
}

pip install gh-pr-comment
FAILED_MESSAGE="[#${TRAVIS_BUILD_NUMBER}] FAILED on ROS ${ROS_DISTRO}"
PASSED_MESSAGE="[#${TRAVIS_BUILD_NUMBER}] PASSED on ROS ${ROS_DISTRO}"

(set -o pipefail; catkin_make ${CM_OPTIONS} 2>&1 | tee ${LOG}) \
  || (gh-pr-comment "${FAILED_MESSAGE}" \
      "\`catkin_make\` failed$(error_log)"; false)
(set -o pipefail; catkin_make install ${CM_OPTIONS} 2>&1 | tee ${LOG}) \
  || (gh-pr-comment "${FAILED_MESSAGE}" \
      "\`catkin_make install\` failed$(error_log)"; false)

source /catkin_ws/devel/setup.bash

(set -o pipefail; catkin_make tests ${CM_OPTIONS} 2>&1 | tee ${LOG}) \
  || (gh-pr-comment "${FAILED_MESSAGE}" \
      "\`catkin_make tests\` failed$(error_log)"; false)
(set -o pipefail; catkin_make run_tests ${CM_OPTIONS} 2>&1 | tee ${LOG}) \
  || (gh-pr-comment "${FAILED_MESSAGE}" \
      "\`catkin_make run_tests\` failed$(error_log)"; false)

result_text="
\`\`\`
`catkin_test_results --all | grep -v Skipping || true`
\`\`\`
"
errored_tests=`catkin_test_results --all | grep -v -e "^Skipping" -v -e "^Summary" | grep -e "[1-9][0-9]* errors" -e "[1-9][0-9]* failures" | cut -d":" -f1`
result_text_detail="
`echo ${errored_tests} | xargs -n 1 -- bash -c 'echo; echo \#\#\# $0; echo; echo \\\`\\\`\\\`xml; xmllint --format $0; echo \\\`\\\`\\\`;'`
"
catkin_test_results || (gh-pr-comment "${FAILED_MESSAGE}" "Test failed$result_text$result_text_detail"; false)

gh-pr-comment "${PASSED_MESSAGE}" "All tests passed$result_text"
