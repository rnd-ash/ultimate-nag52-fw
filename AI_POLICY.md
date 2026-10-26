# AI Usage Policy

A few rules that align with my beliefs:

1. *All AI usage in any form must be disclosed*. You should state the tool you used, along with the extent that AI assisted you.
2. *The human in the loop MUST FULLY understand the code* If you cannot explain what your agent has created, do NOT contribute to the project.
3. *Spamming hundreds of slop PRs will not be tolerated*. As much as an AI can find issues and submit PRs for each issue. These PRs take considerable time to review and fix.

## Just because AI fixed something in your car, doesn't mean it'll fix it for everyone.

Please remember. When AI is helping you fix 'problems' with the TCU code base, it will try and find whatever it can to make YOUR experience better.

Whilst this is a good idea, your car is a sample size of 1, and this TCU has to support 200+ combinations of cars and gearboxes. Therefore, it is likely
that the one fix for your car might make things worse for everyone else.

Therefore, it is imperative that you review what was changed, FULLY understand why this works in your case, and then consider submitting a PR if you
see that this might positively affect others. 

For instance, if the AI is just changing loads of variables in the shifting algorithms, this would likely cause issues for others.

## The use of AI is welcome

AI is a tool, which does have a lot of positive. Therefore, it is welcome here to aid in contributions where necessary.

The strict AI policy comes down to the responsibility I have: To ensure that the code works in EVERYONE'S car, and does not cause any potential damage
to gearboxes. 

The high number of PRs being submitted by agents who have no idea how this gearbox functions or understand the original design concepts of the OEM TCU
are the problem. As this just causes a ton of slop code to be added, which is unmaintainable at best, and dangerous at worse.
