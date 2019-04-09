from argparse import ArgumentParser

import localization.kalman
import localization.kalman_driver
import util


# Run localization algorithm retroactively on csv data.
def localize_retro(gps_filename, bike_filename):
    localization.kalman_driver.draw_retro(
        gps_filename=gps_filename, bike_filename=bike_filename
    )


# Run localization algorithm and compare to old (non-odometry) results.
def localize_compare_retro(gps_filename, bike_filename):
    localization.kalman_driver.kalman_compare_retro(
        gps_filename=gps_filename, bike_filename=bike_filename
    )


# Run live localization algorithm.
def localize():
    raise NotImplementedError()


if __name__ == '__main__':
    parser = ArgumentParser(description='Navigation main executable.')
    parser.add_argument('action',
            help='Which command; possible choices are localize-retro, localize-compare-retro')
    parser.add_argument('--gps-file')
    parser.add_argument('--bike-file')
    args = parser.parse_args()

    if args.action == 'localize-retro':
        localize_retro(args.gps_file, args.bike_file)

    elif args.action == 'localize-compare-retro':
        localize_compare_retro(args.gps_file, args.bike_file)

    elif args.action == 'localize':
        localize()

    else:
        print('command "{}" not recognized'.format(args.action))
