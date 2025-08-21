#!/usr/bin/env bash

echo -e "This script will complete a full release cycle in one step."
echo -e "Only run this if you are sure you want to release a new version to Github"

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
REPO_NAME=$(basename $WD)

unset DO_RELEASE
until [[ $DO_RELEASE =~ [YyNn] ]] ; do
  read -p "Are you sure you want to proceed [N/y]: " DO_RELEASE
  DO_RELEASE=${DO_RELEASE:-N}
done

if [[ $DO_RELEASE =~ [yY] ]]
then
  if scripts/./startRelease.sh ; then
      scripts/./finishRelease.sh
  else
      exit 1
  fi
else
  echo -e "No worries mate!  We can do it another time"
  exit 0
fi