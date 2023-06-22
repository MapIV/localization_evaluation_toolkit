#!/bin/bash

ps -aux | grep ros | awk '{print $2}' | xargs kill -9

