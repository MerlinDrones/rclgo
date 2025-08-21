#!/usr/bin/env bash
set -e

CUR_DIR=`pwd`
WD=`pwd`

if [[ "$WD" = */scripts ]];
then
  echo "Not in Project root.  Changing WD to project root"
  cd ..
  WD=`pwd`

else
  echo "In Project Root"
fi
echo "Working Dir: $WD"

if [[ `git status --porcelain` ]]; then
  # Changes
  echo -e "There are uncommitted changes. Commit these first then try to release"
  exit 0
fi

VERSION=$(< VERSION)
unset RELEASE_TYPE
RELEASE_VERSION=$(semver bump -d minor | tail -1 | awk '{print $NF}')
PATTERN="[m](ajor|inor)|patch"
echo -e "Current Version is" $VERSION
until [[ $RELEASE_TYPE =~ $PATTERN ]] ; do

  read -p "Is this a Major or Minor Version Release [major|minor|patch] (default is \"minor\"): " RELEASE_TYPE
  RELEASE_TYPE=${RELEASE_TYPE:-minor}
done
RELEASE_VERSION=$(semver bump -d $RELEASE_TYPE | tail -1 | awk '{print $NF}')
echo "New release version will be $RELEASE_VERSION"
echo $RELEASE_TYPE
unset CONTINUE_RELEASE
until [[ $CONTINUE_RELEASE =~ [YyNn] ]] ; do
  read -p "Are you sure you want to proceed [n/Y]: " CONTINUE_RELEASE
  CONTINUE_RELEASE=${CONTINUE_RELEASE:-Y}
done

if [[ $CONTINUE_RELEASE =~ [nN] ]]
then
  echo -e "No worries mate!  We can do it another time."
  exit 0
fi

semver bump $RELEASE_TYPE

# ensure you are on latest develop  & master
git checkout master
git pull origin master
git checkout develop
git pull origin develop
echo -e "----------------------------------"
git flow release start $RELEASE_VERSION

#git checkout develop
echo -e "\n-------------------------------------------------------------------------------"
echo -e "Start your testing on this branch"
echo -e "NOTE:  Only docs and bug fixes go on this branch until it is finished"
echo -e "All new features should be off of develop"
echo -e "When you are ready to release run scripts/finishRelease.sh"
echo -e "-------------------------------------------------------------------------------"
exit 0