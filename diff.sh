#!/bin/sh
for file in AirMiniSketchTransmitter/* libraries/*/* doc/*; do echo $file::; diff $file ../../GitHub/AirMiniTransmitter/$file; done
