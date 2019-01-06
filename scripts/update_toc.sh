#!/bin/bash
gh-md-toc --insert README.md
rm README.md.orig.*
rm README.md.toc.*
git add README.md
git status
