#!/bin/sh

./../../build/out/tools/text_tool/text_tool Roboto-Medium.ttf 24 ../../data/fonts/roboto_medium_24
./../../build/out/tools/text_tool/text_tool Roboto-Medium.ttf 64 ../../data/fonts/roboto_medium_64

# gen slug font
./../../build/out/tools/text_tool2/text_tool2 Roboto-Medium.ttf ../../data/fonts/roboto_medium.slug
./../../build/out/tools/text_tool2/text_tool2 Zapfino.ttf ../../data/fonts/zapfino.slug
./../../build/out/tools/text_tool2/text_tool2 TerminusTTF.ttf ../../data/fonts/terminus.slug

