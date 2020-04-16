#!/bin/bash
sudo apt-get install unzip
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1cAfJm_oJd3_u1pES0UQhCSd1eCeszrnT' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1cAfJm_oJd3_u1pES0UQhCSd1eCeszrnT" -O CarlarSimulator.zip && rm -rf /tmp/cookies.txt
unzip CarlarSimulator.zip
