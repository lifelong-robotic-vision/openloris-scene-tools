#!/usr/bin/env python2
import argparse
import pyrealsense2 as rs
import numpy as np
import cv2
import os

def main():
    if args.save_all and not os.path.exists(args.output + '/color'):
        os.mkdir(args.output + '/color')
    if args.save_all and not os.path.exists(args.output + '/depth'):
        os.mkdir(args.output + '/depth')
    if not os.path.exists(args.output + '/aligned_depth'):
        os.mkdir(args.output + '/aligned_depth')
    existing_aligned_depth = {s for s in os.listdir(args.output + '/aligned_depth') if s.endswith('.png')}
    stamp_to_filename = lambda t: '%f.png' % (t * 1e-3)
    try:
        config = rs.config()
        rs.config.enable_device_from_file(config, args.input)
        align = rs.align(rs.stream.color)
        pipeline = rs.pipeline()
        replay = 0
        frame_ids = set()
        pipeline.start(config)
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            frame_id = depth_frame.get_frame_number()
            if replay == 0 or frame_id < last_frame_id:
                # first frame or start over
                if replay > 0:
                    missed = max(frame_ids) - min(min(frame_ids), frame_id) + 1 - len(frame_ids)
                    print ('Last frame %d. Missed %d frames.' % (last_frame_id, missed))
                    if (missed == 0): break
                replay += 1
                print ('Replay #%d: frame ID starts from %d' % (replay, frame_id))
                last_frame_id = frame_id
            elif frame_id == last_frame_id:
                # ignore repetitive frames
                continue
            else:
                last_frame_id += 1
            if replay > 1:
                missed = max(frame_ids) - min(min(frame_ids), frame_id) + 1 - len(frame_ids)
                if (missed == 0): break
            while last_frame_id < frame_id:
                print ("Missed frame %ld" % last_frame_id)
                last_frame_id += 1
            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                print("Invalid frame %ld" % frame_id)
                exit()
            frame_ids.add(frame_id)

            if args.save_all or args.render:
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
            aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
            if args.save_all:
                cv2.imwrite(args.output + '/depth/' + stamp_to_filename(depth_frame.timestamp), depth_image)
                cv2.imwrite(args.output + '/color/' + stamp_to_filename(color_frame.timestamp), color_image)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imwrite(args.output + '/depth_rgb/' + stamp_to_filename(depth_frame.timestamp), depth_colormap)
            if stamp_to_filename(aligned_depth_frame.timestamp) not in existing_aligned_depth:
                cv2.imwrite(args.output + '/aligned_depth/' + stamp_to_filename(aligned_depth_frame.timestamp), aligned_depth_image)

            if args.render:
                # Render images
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                aligned_depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(aligned_depth_image, alpha=0.03), cv2.COLORMAP_JET)
                images = np.hstack((color_image, depth_colormap, aligned_depth_colormap))
                cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Align Example', images)
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
        print ('Saved %d frames (%d ... %d)' % (len(frame_ids), min(frame_ids), max(frame_ids)))
    finally:
        pipeline.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", type=str, help="Bag file to read", default="record-d400.bag")
    parser.add_argument("-o", "--output", type=str, help="Path to save the images", default=".")
    parser.add_argument("-a", "--save-all", type=bool, help="Save color and raw depth images", default=False)
    parser.add_argument("-r", '--render', dest='render', action='store_true')
    parser.set_defaults(render=False)
    args = parser.parse_args()

    main()
