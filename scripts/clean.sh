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
echo -e "Deleting Repo"
gh repo delete $(basename $WD) --yes
rm VERSION || true
rm go.mod || true
rm -rf .git || true

echo -e "DONE"