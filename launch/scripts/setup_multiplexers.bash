#!/bin/bash 

rosrun topic_tools mux /output_image /lk_tracker/output_image_pi /lk_tracker/output_image_pc mux:=mux_lk &

rosrun topic_tools mux /face_detect_output_image/mul_fd face_detect_output_image/face_detect_output_image_pi face_detect_output_image/face_detect_output_image_pc mux:=mux_fd &

rosrun topic_tools mux /image_pre_processing/mul_pp /image_pre_processing/pre_processed_image_pi /image_pre_processing/pre_processed_image_pc mux:=mux_pp &

rosrun topic_tools mux /face_box_coordinates/mul_fbc /face_box_coordinates/face_box_coordinates_pi /face_box_coordinates/face_box_coordinates_pc mux:=mux_fbc

