# MSTE
Team Members:  
  
  - Yubo Wang
  - Vaibhav Anand
  - Nathan Bowler
  - Sarah Lange
  - Zehui Xiao
  - Declan Coster
  - Brian Zhu
  - Gustavo Cardona
  - Disha Kamale
  - Zayd Aldahleh
  
  
 ## Summary of current PRs: 
 
 1. [Feature/dynamic purepursuit](https://github.com/wbriang/MTSE/pull/13):

  - Studies the speed depdendence on look-ahead distance.
  - Currently has some conflicts with master that need to be resolved before commiting new changes. 

2. [Fix/prepursuitintegration](https://github.com/wbriang/MTSE/pull/12):

  - Currently blocked; other PRs need to be merged first. 

3. [Feature/goalplanner](https://github.com/wbriang/MTSE/pull/11): 

  - Ready to be tested and then merged. 


## Summary of branches:

1. [feature/rrt](https://github.com/wbriang/MTSE/tree/feature/rrt):

  - Contains the path planning codes
  - Current status: Not working properly 
  - Needs to be merged after correcting 
  
2. [integration/fullnavigationstack](https://github.com/wbriang/MTSE/tree/integration/fullnavigationstack):

  - contains a cleaned-up version of [tmp/nav_stack_integration](https://github.com/wbriang/MTSE/tree/tmp/nav_stack_integation)

3. [feature/savelidarinfo](https://github.com/wbriang/MTSE/tree/feature/savelidarinfo):

  - Current status: issues converting the pointcloud data to Laser_Scan message. 
 
4. [feature/main_launch](https://github.com/wbriang/MTSE/tree/feature/main_launch): 

  - Current status: Contains commits from both main and rrt branches. The rrt commits either need to be reverted or need to create another branch and start over with main_launch files. 


TODOs: The branches [feature/goalplanner](https://github.com/wbriang/MTSE/tree/feature/goalplanner), [feature/localizationAMCL](https://github.com/wbriang/MTSE/tree/feature/localizationAMCL) and [feature/dynamic_purepursuit](https://github.com/wbriang/MTSE/tree/feature/dynamic_purepursuit) need tp integrated into master before making other (huge) integrations. 
