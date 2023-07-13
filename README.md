# Jackal Kimera Sim

This repo contains some things relating to try and run kimera on a jackal in a gazebo simulation but also some scripts for testing the accuracy of Kimera.

## Kimera Accuracy Scripts

There are 2 important files in here for this `kimera_data_collector.py` and `kimera_data_analyzer.py`.

### Collecting data

You can use `kimera_data_collector.py` to collect both truth and estimate data for a given test. Right now it is set up for the Euroc dataset. In the file you can change the name of the test you are doing at the top of the file. Data will be saved to a file according to this name so it is importnant to change between runs. To run it, start kimera and then run

```
./kimera_data_collector.py
```

This will have it start listening for truth and estimate data. Then start the euroc bag file and let Kimera run. Before the bag file is done publish to the `/save` topic so it stops collecting data:
```
rostopic pub /save std_msgs/Bool "data: false"
```

This will save the truth and estimate data in a .npy file which can be analyzed later.


### Analyzing data

You can use `kimera_data_analyzer.py` to anlyze your data. First change the name at the top of the file to match the name you used when collecting your data. Then simply run:

```
python kimera_data_analyzer.py
```

At this time this will just align the frames of the estimate and truth data and then graph the 2 against each other to be compared visually.
