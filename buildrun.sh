#! /bin/bash

dts devel build -f -H kigebot.local --privileged

dts devel run -H kigebot.local -- --privileged
