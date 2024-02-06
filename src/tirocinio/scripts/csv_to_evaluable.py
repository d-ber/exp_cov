# rostopic echo /base_pose_ground_truth --noarr -p [> gt_in.csv]:
#%time,field.header.seq,field.header.stamp,field.header.frame_id,field.child_frame_id,field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w,field.twist.twist.linear.x,field.twist.twist.linear.y,field.twist.twist.linear.z,field.twist.twist.angular.x,field.twist.twist.angular.y,field.twist.twist.angular.z
#1579700000000,15796,1579700000000,odom,,-6.54828973954178,8.67876889974828,0.0,0.0,0.0,-0.0706767257524745,0.9974992734016951,0.0,0.0,0.0,0.0,0.0,0.0
# rostopic echo /odom --noarr -p [> odom_in.csv]:
#%time,field.header.seq,field.header.stamp,field.header.frame_id,field.child_frame_id,field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w,field.twist.twist.linear.x,field.twist.twist.linear.y,field.twist.twist.linear.z,field.twist.twist.angular.x,field.twist.twist.angular.y,field.twist.twist.angular.z
#21909600000000,219095,21909600000000,odom,,10.366380365399005,8.450856603709378,0.0,0.0,0.0,-0.9943559000554434,0.10609591898338513,0.0,0.0,0.0,0.0,0.0,0.0

import csv
import argparse
import os

def parse_args():
    parser = argparse.ArgumentParser(description='Make csv odom data ok for evaluation scripts. csv is assumed to be in format %time,field.header.seq,field.header.stamp,field.header.frame_id,field.child_frame_id,field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w,field.twist.twist.linear.x,field.twist.twist.linear.y,field.twist.twist.linear.z,field.twist.twist.angular.x,field.twist.twist.angular.y,field.twist.twist.angular.z')
    parser.add_argument('-i', '--input', required=True,
        help="Path to the input csv file.", metavar="CSV_PATH")
    parser.add_argument('-o', '--output', default="out",
        help="Name of the output csv file. Default is 'out'", metavar="NAME")
    parser.add_argument("-d", '--dir', default=os.getcwd(),
        help="Base directory to save files in. Default is cwd.")
    return parser.parse_args()

def process_csv(input_filepath, output_filepath):
    print(output_filepath)
    with open(output_filepath, 'w') as output_file:
        writer = csv.writer(output_file)
        with open(input_filepath) as input_file:
            reader = csv.reader(input_file, delimiter=",")
            next(reader) # skip header
            for row in reader:
                timestamp = float(row[0])
                tx, ty, tz = float(row[5]), float(row[6]), float(row[7])
                qx, qy, qz, qw = float(row[8]), float(row[9]), float(row[10]), float(row[11])
                writer.writerow([timestamp, tx, ty, tz, qx, qy, qz, qw])

def main():

    args = parse_args()
    process_csv(args.input, os.path.join(args.dir, args.output + ".csv"))

                
if __name__ == '__main__':
    main()