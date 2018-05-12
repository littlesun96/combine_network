import os
import math
from subprocess import call
import xml.etree.ElementTree as ET
from xml.dom import minidom
from multiprocessing.dummy import Pool as ThreadPool

# -----------------------------------------------------------------------------


class TPoint:
    def __init__(self, pid, x, y, z, time_stamp, width, height):
        self.pid = pid
        self.x = x
        self.y = y
        self.z = z
        self.time_stamp = time_stamp
        self.width = width
        self.height = height
        self.ptype = 2
        self.features = []
# -----------------------------------------------------------------------------


class Trajectory:
    def __init__(self, pid, start_frame, ptype):
        self.pid = pid
        self.start_frame = start_frame
        self.ptype = ptype
        self.points = []
# -----------------------------------------------------------------------------


class TFileLine:
    def __init__(self, pid, ptype, x, y, z, time_stamp, rankc):
        self.pid = pid
        self.ptype = ptype
        self.x = x
        self.y = y
        self.z = z
        self.time_stamp = time_stamp
        self.rankc = rankc

    def make_str(self, ind, delim):
        sline = ''
        sline += str(ind) + delim + str(self.pid) + delim + str(self.ptype) + delim + str(self.x) + delim + str(self.y) + delim + str(self.z) + delim + str(self.time_stamp) + delim + str(self.rankc) + '\n'
        return sline
# -----------------------------------------------------------------------------


def make_line(tp, features, delim):
    sline = str(tp) + delim
    for f in features:
        sline += f + delim
    sline = sline[:-1]
    # print('line length is ', str(len(sline)))
    sline += '\n'
    return sline

# -----------------------------------------------------------------------------


def is_intersect(x1_1, x2_1, x1_2, x2_2):
    w = 0
    if x1_1 <= x1_2:
        if x2_1 >= x1_2:
            w = min(x2_1, x2_2) - x1_2
    else:
        if x1_1 <= x2_2:
            w = min(x2_1, x2_2) - x1_1

    return w
# -----------------------------------------------------------------------------


def parse_txt(file_path, id_prefix, video_traj_ann, result_file):
    global generate_mode

    print('parse_txt: ', file_path)

    video_traj = {}

    tfile = open(file_path, "r")

    counter = 0

    frame_width = 0
    frame_height = 0

    draw_list = {}

    try:

        for line in tfile:
            if len(line) > 0:
                counter += 1

            if counter == 1:
                values = line.split(',')
                if len(values) == 2:
                    frame_width = int(values[0])
                    frame_height = int(values[1])
                    # print('begin')

            values = line.split(',')
            if len(values) == 12 or len(values) == 1052:
                start_frame = int(values[0])
                pid = int(values[1])
                ptype = int(values[2])
                x = float(values[3])
                y = float(values[4])
                z = float(values[5])
                width = float(values[6])
                height = float(values[7])
                time_stamp = int(values[8])
                # ind = int(values[9])

                pt = TPoint(pid, x, y, z, time_stamp, width, height)

                if len(values) == 1052:
                    # print('bingo!')
                    pt.features = values[12:1052]

                if pid in video_traj:
                    video_traj[pid].points.append(pt)
                else:
                    tr = Trajectory(pid, start_frame, ptype)
                    tr.points.append(pt)
                    video_traj[pid] = tr

                if start_frame in draw_list:
                    draw_list[start_frame].append(pt)
                else:
                    points = [pt]
                    draw_list[start_frame] = points

        tfile.close()

        # Search quadrocopter in handmade annotations
        for traj_auto in video_traj.values():
            x1_1 = traj_auto.start_frame
            x2_1 = traj_auto.start_frame + len(traj_auto.points)

            for traj_ann in video_traj_ann.values():
                x1_2 = traj_ann.start_frame
                x2_2 = traj_ann.start_frame + len(traj_ann.points)

                int_w = is_intersect(x1_1, x2_1, x1_2, x2_2)
                if int_w > 0:
                    from_auto = 0
                    from_ann = 0
                    if x1_1 < x1_2:
                        from_auto = x1_2 - x1_1
                    elif x1_2 < x1_1:
                        from_ann = x1_1 - x1_2

                    # print('Intersect area = ', int_w, ', auto = [', x1_1, ', ', x2_1, '], ann = [', x1_2, ', ', x2_2, '], from_auto = ', from_auto, ', from_ann = ', from_ann)
                    right_count = 0
                    for i in range(int_w):
                        # print('from_auto + i = ', from_auto + i, ' = ', len(traj_auto.points), ', from_ann + i = ', from_ann + i, ' = ', len(traj_ann.points))
                        pt_auto = traj_auto.points[from_auto + i]
                        pt_ann = traj_ann.points[from_ann + i]
                        dist = math.sqrt((pt_auto.x - pt_ann.x) * (pt_auto.x - pt_ann.x) + (pt_auto.y - pt_ann.y) * (pt_auto.y - pt_ann.y))
                        thresh = 5 * math.sqrt(pt_ann.width * pt_ann.width + pt_ann.height * pt_ann.height)
                        if dist < thresh:
                            right_count = right_count + 1

                    # print('right_count = ', right_count, ' and int_w / 2 = ', int_w / 2, ', traj_auto.pid = ', traj_auto.pid, ', traj_ann.pid = ', traj_ann.pid)
                    if right_count > int_w / 2:
                        traj_auto.ptype = 1

            if traj_auto.ptype == 1:
                for i in range(x1_1, x2_1):
                    if i in draw_list:
                        # print('i = ', i, ' and x1_1 = ', x1_1, ', x2_1 = ', x2_1, 'len(draw_list[i]) = ', len(draw_list[i]), ', traj_auto.pid = ', traj_auto.pid)
                        for j in range(len(draw_list[i])):
                            if (draw_list[i][j].pid == traj_auto.pid):
                                # print('draw_list[i][j].pid = ', draw_list[i][j].pid)
                                draw_list[i][j].ptype = 1

        for traj_ann in video_traj_ann.values():
            i = traj_ann.start_frame
            for pt in traj_ann.points:
                pt.ptype = 3
                i += 1

        print('parse result: ', len(video_traj))

        if generate_mode:
            for traj in video_traj.values():
                new_id = id_prefix + '_' + str(traj.pid)
                traj_type = 2
                if traj.ptype == 1:
                    traj_type = 1
                i = 1
                # print('converting: trajectory type = ', traj.ptype)
                for pt in traj.points:
                    # line = TFileLine(new_id, traj.ptype, pt.x, pt.y, pt.z, pt.time_stamp, i)
                    # Normalize coordinates
                    # if frame_width > 0 and frame_height > 0:
                    #         line.x = line.x / frame_width
                    #         line.y = line.y / frame_width
                    # result_file.write(line.make_str(i, ','))
                    if len(pt.features) != 0:
                        result_file.write(make_line(traj_type, pt.features, ','))
                    i = i + 1

    except Exception as e:
        print('Parse txt error: ', str(e))
        pass

    return draw_list
# -----------------------------------------------------------------------------


def parse_xml(file_path):

    video_traj = {}

    try:
        tree = ET.parse(file_path)
        root = tree.getroot()

        time_stamp = 0
        quadro_type = 1

        for frame in root.findall('frame'):
            frame_ind = int(frame.get('ind'))

            for obj in frame.findall('object'):
                obj_type = obj.get('type')
                if obj_type == 'quadrocopter':
                    obj_id = int(obj.get('ind'))
                    obj_rect = obj.find('rect')
                    x = float(obj_rect.get('x'))
                    y = float(obj_rect.get('y'))
                    width = float(obj_rect.get('width'))
                    height = float(obj_rect.get('height'))

                    pt = TPoint(obj_id, x, y, width / height, time_stamp, width, height)

                    if obj_id in video_traj:
                        video_traj[obj_id].points.append(pt)
                    else:
                        tr = Trajectory(obj_id, frame_ind, quadro_type)
                        tr.points.append(pt)
                        video_traj[obj_id] = tr

    except Exception as e:
        print('Parse xml error: ', str(e))

    return video_traj
# -----------------------------------------------------------------------------


class VideoParams:
    def __init__(self, file_path, start_frame, end_frame):
        self.file_path = file_path
        self.start_frame = start_frame
        self.end_frame = end_frame
# -----------------------------------------------------------------------------


def save_rects_xml(in_file_name, res_file_name, draw_list):

    print('save_rects_xml:\n draw_list = ', len(draw_list), ', in_file_name = ', in_file_name, ', res_file_name = ', res_file_name)

    # <video name="16_05_29.723.avi" camera_type="color" time="day" season="summer">
    #    <frame ind="0" filename="0" area="land" camera_type="color" time="day" season="summer">
    #        <object ind="0" type="quadrocopter" mark_type="manual" calc_type="auto">
    #            <rect x="1256" y="224" width="12" height="13"/>
    #        </object>
    #    </frame>

    root = ET.Element('video', name=in_file_name, camera_type='color', time='day', season='summer')

    try:
        max_ind = 0
        for key in draw_list:
            if max_ind < key:
                max_ind = key
        print('max_ind = ', max_ind)

        for ind in range(max_ind):
            frame = ET.SubElement(root, 'frame', ind=str(ind), filename='0', area='land', camera_type='color', time='day', season='summer', work_time='1')

            if ind in draw_list:
                for pt in draw_list[ind]:
                    otype = 'other'
                    if pt.ptype == 1:
                        otype = 'auto_type'
                    elif pt.ptype == 3:
                        otype = 'quadrocopter'
                    obj = ET.SubElement(frame, 'object', ind=str(pt.pid), type=otype, mark_type='manual', calc_type='auto')
                    rr = ET.SubElement(obj, 'rect', x=str(pt.x), y=str(pt.y), width=str(pt.width), height=str(pt.height))

    except Exception as e:
        print('save_rects_xml while error:', str(e))
        pass

    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent='   ')
    with open(res_file_name, 'w') as f:
        f.write(xmlstr)
# -----------------------------------------------------------------------------


def apply_func(video_param):

    global executable
    global generate_mode

    base_video = os.path.basename(video_param.file_path)
    abs_path = os.path.dirname(video_param.file_path)

    if generate_mode:
        result_file = open(base_video + '.dataset.csv', "w")
        # result_file.write(',id,EVENTMSGTYPE,x,y,z,game_clock,rankc\n')

    try:
        # Run detector for trajectories generate
        args = [executable, video_param.file_path, '-sf=' + str(video_param.start_frame), '-ef=' + str(video_param.end_frame), '-e=1', '-ed=1', '-sl=0']
        print(args)
        if generate_mode:
            call(args)

        # Parse handmade annotations
        ann_file_name = abs_path + os.sep + 'annotations_' + base_video + '.xml'
        video_traj_ann = {}
        if os.path.exists(ann_file_name):
            print('Annotations exist: ', ann_file_name)
            video_traj_ann = parse_xml(ann_file_name)
        else:
            print('Annotations don\'t exist: ', ann_file_name)

        # Parse trajectories
        draw_list = parse_txt(video_param.file_path + '_' + str(video_param.start_frame) + '.txt', base_video + '_' + str(video_param.start_frame), video_traj_ann, result_file)

        save_rects_xml(video_param.file_path, video_param.file_path + '_' + str(video_param.start_frame) + '.xml', draw_list)

    except Exception as e:
        print('General loop error:', str(e))
        pass

    if generate_mode:
        result_file.close()
# -----------------------------------------------------------------------------


def main():
    files = []
    # /media/snuzhny/75fd8728-07e7-44b6-aa2b-d59c12c63ece
    # /home/snuzhny/work

    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\0\\151223-2202.mp4', 0, 0))
    #
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\15_59_34.072.avi', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\16_00_06.789.avi', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\16_00_33.567.avi', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\16_01_21.992.avi', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\16_01_41.812.avi', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\16_01_58.060.avi', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\16_02_19.232.avi', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\16_02_38.857.avi', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\16_04_10.661.avi', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\16_05_29.723.avi', 0, 0))

    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\2\\uav_rec4.mp4', 0, 1200))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\2\\uav_rec4.mp4', 1275, 2500))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\2\\uav_rec4.mp4', 2750, 7000))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\2\\uav_rec4.mp4', 8650, 10500))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\2\\uav_rec4.mp4', 10675, 11100))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\2\\uav_rec4.mp4', 11375, 11675))

    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\3\\flight.mp4', 0, 0))

    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\4\\birds.mp4', 5000, 5500))

    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\5\\test.mp4', 0, 0))

    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\6\\snow.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\6\\snow2.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\6\\snow3.mp4', 0, 0))

    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\7\\12.04.2017_1135_0000.avi', 0, 1250))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\7\\12.04.2017_1135_0000.avi', 1325, 2250))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\7\\12.04.2017_1135_0000.avi', 2350, 0))
    #
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\birds_and_ann/b1.mp4', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\birds_and_ann/b2.mp4', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\birds_and_ann/b3.mp4', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\birds_and_ann/b4.mp4', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\birds_and_ann/b5.mp4', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\birds_and_ann/b6.mp4', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\birds_and_ann/b7.mp4', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\birds_and_ann/b8.mp4', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\birds_and_ann/b9.mp4', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\1\\birds_and_ann/b10.mp4', 0, 0))
    # files.append(VideoParams('/media/snuzhny/75fd8728-07e7-44b6-aa2b-d59c12c63ece/video/uav_detection/8/birds_and_ann/b11.mp4', 0, 0))
    # files.append(VideoParams('/media/snuzhny/75fd8728-07e7-44b6-aa2b-d59c12c63ece/video/uav_detection/8/birds_and_ann/b12.mp4', 0, 0))
    # files.append(VideoParams('/media/snuzhny/75fd8728-07e7-44b6-aa2b-d59c12c63ece/video/uav_detection/8/birds_and_ann/b13.mp4', 0, 0))
    # files.append(VideoParams('/media/snuzhny/75fd8728-07e7-44b6-aa2b-d59c12c63ece/video/uav_detection/8/birds_and_ann/b14.mp4', 0, 0))
    # files.append(VideoParams('/media/snuzhny/75fd8728-07e7-44b6-aa2b-d59c12c63ece/video/uav_detection/8/birds_and_ann/b15.mp4', 0, 0))
    # files.append(VideoParams('/media/snuzhny/75fd8728-07e7-44b6-aa2b-d59c12c63ece/video/uav_detection/8/birds_and_ann/b16.mp4', 0, 0))
    # files.append(VideoParams('/media/snuzhny/75fd8728-07e7-44b6-aa2b-d59c12c63ece/video/uav_detection/8/birds_and_ann/b17.mp4', 0, 0))
    # files.append(VideoParams('/media/snuzhny/75fd8728-07e7-44b6-aa2b-d59c12c63ece/video/uav_detection/8/birds_and_ann/b18.mp4', 0, 0))
    # files.append(VideoParams('/media/snuzhny/75fd8728-07e7-44b6-aa2b-d59c12c63ece/video/uav_detection/8/birds_and_ann/b19.mp4', 0, 0))
    # files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\8\\qc.mp4', 0, 0))

    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\9\\gopro142.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\9\\gopro143.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\9\\gopro144.mp4', 0, 0))

    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\10\\877_frag_one.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\10\\877_frag_two.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\10\\877_frag_three.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\10\\877_frag_four.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\10\\877_frag_five.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\10\\877_frag_six.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\10\\877_frag_seven.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\10\\877_frag_eight.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\10\\888.mp4', 0, 0))

    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\11\\10.4.14.84_19-10-2017_15-58-28.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\11\\Cam4_19-10-2017_14-30-13.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\11\\Cam4_23-10-2017_17-20-03.mp4', 0, 0))

    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\12\\Cam_4_03-11-2017_12-23-32.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\12\\Cam_4_03-11-2017_12-34-08.mp4', 0, 0))
    #
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\13\\Phantom3_10.4.14.82_03-11-2017_19-32-21.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\13\\Phantom3_10.4.14.82_03-11-2017_19-34-26.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\13\\Phantom3_10.4.14.82_03-11-2017_19-36-48.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\13\\Phantom3_10.4.14.82_03-11-2017_19-38-57.mp4', 0, 0))
    #
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\13\\Phantom3_10.4.14.82_03-11-2017_19-40-15.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\13\\Phantom3_10.4.14.82_03-11-2017_19-41-36.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\13\\Phantom3_10.4.14.82_03-11-2017_19-57-07.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\13\\Phantom3_10.4.14.82_03-11-2017_19-57-24.mp4', 0, 0))
    #
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\17\\81_1.mp4', 0, 0))
    files.append(VideoParams('D:\\Usacheva\\video\\uav_detection\\17\\84_1.mp4', 0, 0))


    files_test = []

    print(len(files))

    pool = ThreadPool(2)
    pool.map(apply_func, files)
# -----------------------------------------------------------------------------

# executable = '/home/yulia/work/save_data/x64/bin/detect_track'
executable = "D:\\Usacheva\\uav\\matching\\matching_test_build\\bin\\Release\\detect_track.exe"
generate_mode = True
gl_line_ind = 0

if __name__ == "__main__":
    main()
