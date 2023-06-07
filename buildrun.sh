#! /bin/bash

dts devel build -f -H taobot.local --privileged

dts devel run -H taobot.local -- --privileged
