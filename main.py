#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 14 15:31:19 2019q

@author: pi
"""
from videostream import VideoStream
from videoprocess import VideoProcess
from navigation_old_target import Navigation
import time
import cv2

if __name__ == '__main__':


    vision = VideoProcess(debug=True, ball=True, obstacles=True, wall=True)
    nav = Navigation()
    stream = VideoStream().start()

    time.sleep(2)

    try:
        while True:

            frame = stream.read()

            if frame is None:
                break

            # (obs1, obs2, obs3), goal, ball
            (obs), (ball), (goal), (wall) = vision.update(frame)

            # only true if ball is kicked
            if nav.update(obs, ball, goal, wall):
                print("ball kicked, sleeping for 5 secs")
                time.sleep(5)             
  

            cv2.imshow("pi cam", vision.get())

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        stream.stop()
        nav.stop()

    finally:
        cv2.destroyAllWindows()
        stream.stop()
        nav.stop()
