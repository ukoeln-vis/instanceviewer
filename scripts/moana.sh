#!/bin/bash

ulimit -S -n 200000

export M_MMAP_THRESHOLD=0

export IS=/space/models/moana/island

vglrun -c proxy /home/zellmann/InstanceViewer/build/InstanceViewer \
    $IS/json/isBayCedarA1/isBayCedarA1.json \
    $IS/json/isBeach/isBeach.json \
    $IS/json/isCoastline/isCoastline.json \
    $IS/json/isCoral/isCoral.json \
    $IS/json/isDunesA/isDunesA.json \
    $IS/json/isDunesB/isDunesB.json \
    $IS/json/isGardeniaA/isGardeniaA.json \
    $IS/json/isHibiscus/isHibiscus.json \
    $IS/json/isHibiscusYoung/isHibiscusYoung.json \
    $IS/json/isIronwoodA1/isIronwoodA1.json \
    $IS/json/isIronwoodB/isIronwoodB.json \
    $IS/json/isKava/isKava.json \
    $IS/json/isLavaRocks/isLavaRocks.json \
    $IS/json/isMountainA/isMountainA.json \
    $IS/json/isMountainB/isMountainB.json \
    $IS/json/isNaupakaA/isNaupakaA.json \
    $IS/json/isPalmDead/isPalmDead.json \
    $IS/json/isPalmRig/isPalmRig.json \
    $IS/json/isPandanusA/isPandanusA.json \
    $IS/json/osOcean/osOcean.json
