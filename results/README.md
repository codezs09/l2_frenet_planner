
This folder lists the files of the results.

To repeat the results or draw plots, move the data.bin file back to build/ directory, and then run:

``` 
python3 scripts/fot_v2.py --skip_fot
```


Note: 

To simulate the slow down scenario, you can hardcode the s_flw_vec in the following code block in fot_v2.cpp:

```
 if (fot_ic.lane_id == 1) {
    // std::cout << "s_flw_vec: ";
    // for (auto s_flw : *s_flw_vec) {
    //   std::cout << s_flw << ", ";
    // }
    // std::cout << std::endl;

    // NOTE: hardcode here for slow down scenario to observe if planning
    // drifting.
    s_flw_vec->clear();
    s_flw_vec->push_back(72.5);
  }
```