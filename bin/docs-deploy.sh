#!/bin/bash

# Setup local environment:
cd `dirname $0`/..
if [ ! -f config.toml ]; then
    cp ./bin/default-config.toml ./config.toml
fi

#get varibale from config.toml
WEBPAGES_MODULE=`toml get webpages.module`
WEBPAGES_REPO=`toml get webpages.target`

# Scripts actions:
echo ""
echo "DEPLOY $WEBPAGES_MODULE ON $WEBPAGES_REPO"
echo ""

echo "--Build"
mkdocs build

echo ""
echo "--Copy"
if [ -d $WEBPAGES_REPO/$WEBPAGES_MODULE ]; then
    rm -fr $WEBPAGES_REPO/$WEBPAGES_MODULE
fi

git -C $WEBPAGES_REPO pull

mv site $WEBPAGES_REPO/$WEBPAGES_MODULE

echo ""
echo "--Commit"
git -C $WEBPAGES_REPO add $WEBPAGES_MODULE
git -C $WEBPAGES_REPO commit -m "update from $WEBPAGES_MODULE"
git -C $WEBPAGES_REPO push
