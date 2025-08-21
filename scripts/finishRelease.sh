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

VERSION=$(< VERSION)
# PREVENT INTERACTIVE MERGE MESSAGE PROMPT AT A FINAL STEP
GIT_MERGE_AUTOEDIT=no
export GIT_MERGE_AUTOEDIT
GITBRANCHFULL=`git rev-parse --abbrev-ref HEAD`
GITBRANCH=`echo "$GITBRANCHFULL" | cut -d "/" -f 1`
RELEASETAG=`echo "$GITBRANCHFULL" | cut -d "/" -f 2`
echo $GITBRANCH
echo -e "Current Version: $VERSION"
if [ $GITBRANCH != "release" ] ; then
   echo "Release can be finished only on release branch!"
   exit 1
fi
if [ -z $RELEASETAG ]
then
  echo We expect gitflow to be followed, make sure release branch called release/x.x.x
  exit 1
fi

# ensure you are on latest develop  & master and return back
git checkout develop
git pull origin develop
git checkout -
git checkout master
git pull origin master
git checkout -

git flow release finish -m "release/$RELEASETAG" $RELEASETAG
git push origin develop && git push origin master --tags

unset DO_ASSETS
until [[ $DO_ASSETS =~ [YyNn] ]] ; do
    read -p "Do you want to build and push release assets to github [N/y]: " DO_ASSETS
    DO_ASSETS=${DO_ASSETS:-N}
done
if [[ $DO_ASSETS =~ [yY] ]]
then
  make dist
  ## Use GH CLI to actually create the release on Github
  gh release create $RELEASETAG dist/test-$RELEASETAG-*.tar* --generate-notes
fi

echo -e "\n********************************************************************************************************"
echo -e "                                  Release complete"
echo -e "********************************************************************************************************"
exit 0