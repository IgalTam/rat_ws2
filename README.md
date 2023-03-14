# Welcome to the Nuada codebase
##### [WIP]: Navigate to each directory under ```/src``` to see more specialized READMEs

### Getting Started
Clone this remote repo into your local device. If you have been assigned an issue, locally checkout the development branch for that issue.

### Taking on an Issue
To access your assigned issues, navigate to the ```Issues``` tab in the resource bar, and filter by assignee by selecting the ```Assignee``` dropdown.<br />

When you are ready, on the issue page, click on the hyperlink in ```Development```. Keep the branch name unchanged, but keep note of it as that will be your development branch. Then, copy the popup commands into your local repository, while within the root directory. If you lose them, they are:
```
git fetch origin
git checkout <name of branch>
```
Commit and push your work for that issue onto the issue-specific branch.

### Using the GitHub Project
##### [NOTE: This feature was not used during the Nuada Capstone development cycle, but may be useful for future teams]<br />
To better communicate task/issue allocation and progress, please include your task as an item in the Nuada GitHub Project. Task items should be formatted as follows:<br />

1) Required attributes:<br />
  a) Task Name<br />
  b) Assignee(s)<br />
  c) Deadline<br />
  d) Weekly Meeting Mention: Please list all weekly meeting dates that the task should be referenced on (this will simplify the weekly meeting leader's job of including these tasks in the meeting agenda) (example: "01/19, 01/26")

2) Optional attributes (but still helpful for Gantt chart organizer):<br />
  a) Estimated time (self-explanatory)<br />
  b) Actual time (self-explanatory; for your convenience, this field should be updated dynamically as progress is made on the task)<br />
  c) Labels: if you want to turn the task item into a GitHub issue, please attach any relevant labels to it from the list of labels on the Nuada GitHub repository<br />

The item boxes in the "Nuada Graphic" tab are designated as follows:<br />
  - Triage: planned tasks for a future point in the project (i.e., will be assigned later on)<br />
  - Todo: directly assigned tasks that have not had progress<br />
  - In Progress: directly assigned tasks that do have progress<br />
  - In Review: directly assigned tasks that have been deemed completed by their assignee and are up for being reviewed by the team<br />
  - Done: completed and reviewed tasks<br />
  - Deprecated: tasks that have been designated as infeasible or unnecessary to the project but have developed features that may be useful for other tasks<br />
