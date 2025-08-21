#!/usr/bin/bash
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
REPO_NAME=$(basename $WD)

#Initialize Git
echo -e "\nInitializing Git"
git init

getRepoName() {
  unset REPO_OWNER
  read -p "Repo Owner name (aka Github handle): " REPO_OWNER
  unset CONFIRM_REPO_NAME
  until [[ $CONFIRM_REPO_NAME =~ [YyNn] ]] ; do
    echo -e "Repo: github.com/$REPO_OWNER/$REPO_NAME"
    read -p "Is this correct [n/Y]: " CONFIRM_REPO_NAME
    CONFIRM_REPO_NAME=${CONFIRM_REPO_NAME:-Y}
  done
}

createRepo() {
      REPO_NAME="$REPO_OWNER/$REPO_NAME"
      echo -e "Creating Repo"
      echo -e "Repo Name: $REPO_NAME"
      RESULT=$(gh repo create $REPO_NAME --public)
      wait
      echo -e "Created Repo $RESULT"
      git remote add origin git@github.com:$REPO_NAME.git
      wait
      git remote -v
}

unset CREATE_REPO
until [[ $CREATE_REPO =~ [YyNn] ]] ; do
  read -p "Do you want to create a GitHub repo for this project [N/y]: " CREATE_REPO
  CREATE_REPO=${CREATE_REPO:-N}
done
if [[ $CREATE_REPO =~ [yY] ]]
then
  getRepoName
  if [[ $CONFIRM_REPO_NAME =~ [yY] ]]
  then
    createRepo
  else
    getRepoName
    createRepo
  fi
fi

#Initialize GitFlow
echo -e "\nInitializing Git Flow"
git flow init

#Initialize semver
echo -e "\nInitializing semver"
$(which semver) init
wait

#Initialize go mod
go mod init github.com/$REPO_NAME || true

if [[ $CONFIRM_REPO_NAME =~ [yY] ]]
then
  #Initialize Master Branch on Repo
  git checkout master
  git add README.md LICENSE
  git commit -m "Master branch repo initialization"
  git push origin master

  # Perform Initial Commit on Develop Branch
  git checkout develop
  git pull origin master
  git add .
  git commit -m "Initial commit on develop"
  git push origin develop
fi


echo -e "\n********************************************************************************************************"
echo -e "                                  Project Initialization complete"
echo -e "********************************************************************************************************"
