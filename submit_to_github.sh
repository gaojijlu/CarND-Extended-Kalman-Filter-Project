#!/bin/bash

echo 'check status'
git status
echo 'add to stage'
git add .
echo 'commit to local repo'
git commit -m "xxxxxx"


echo 'set new origin '
git remote set-url origin $upstreamVar

echo 'push to origin'
git push -u origin master


