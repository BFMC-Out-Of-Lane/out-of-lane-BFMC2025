from Brain.src.decision.decision_making.gps.gps import GPS

if __name__ == '__main__':
    gps = GPS('racetrack/test_track_w_roundabout_new.graphml', "37", "42")
    print(gps.path)
