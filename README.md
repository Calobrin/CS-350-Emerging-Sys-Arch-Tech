# CS-350-Emerging-Sys-Arch-Tech
Andrew McPherson
The repository for my work in CS-350 for term 21EW1

# (Primarily focusing on my thermostat project for these answers)
# Summarize the project and what problem it was solving. 
The project was using a GPIO interrupt import to create a mock-up thermostat. The board would be reading the temperature in a room and saving that temperature to a variable.
That variable would then be compared to a setpoint temperature, and if the setpoint was higher than the read temperarure, it would enable an LED and change the "heat" to 1, indicating it is on.

This all ran on a timer, with a timer period of 100ms, but each event needed to be happening at different intervals. Every 200ms it would check if the buttons to change the setpoint weere pressed. Every 500ms it would read the temperature. Every 1000ms (1 second) it would increment the seconds timer and display the information of the current temp, setpoint, if the heat was on, and the current second.

The purpose of the project was to implement a task scheduler so not all events happened at the same time, thus the timer.

# What did you do particularly well?
I think the thind I did the best was beginning to understand how to work with the board. Working on these projects was with technology I was wholly unfamiliar with, but by the end I really started to get a grasp of understanding how everything works, and how I was able to set up the appropriate task scheduler to work with the board.
I am, if nothing else, proud of my ability to learn to work with something I have never used before and begin to get a good understanding of it all.

Other things I think I did well, for the morse code project, was setting up my state machine, and creating various functions to blink the LED efficiently. Using functions allowed the code to be modular, and instead of having each state machine run the blinking LED, it simply called the functions that did so in a specific order.

# Where could you improve?
Based on feedback I received, I should have used constants for the timer interval instead of just numbers, there was probably a more efficient way of doing things, and using hard coded values for my task scheduler intervals was a remnant of early work on the project where I was testing things to see if it worked. While yes, in this instance it worked fine, there could be a better way to do it to make it more reliable and reusable in more situations

# What tools and/or resources are you adding to your support network?
Unsure what this question means as I am unsure what my support network is or means.
I know, for tools, that I can now use my CC3220SF launchpad board to work on things, as I now own one and have spent some time working with it and beginning to understand how it all works, I could begin using it in the future.
but resources added to my support network? I used the resources available from TI, the manual offers a good documentation resource to understand the board. Aside from that probably some C/C++ reference guides for various things when it comes to coding.

# What skills from this project will be particularly transferable to other projects and/or course work?
The skills from this project will be transferable to course work because I will be more than likely working with C or C++ in the future.
It is also potentially possible I may be seeing some of the same pieces of this project in the future, I may work with timers, or interrupts or some of the other types of programs in the future, and my experience working with, and understanding, these types of programs can be helpful
These skills are also transferable because both of these projects used the same base but do entirely different things, just depending on some small changes I made, so in a way it shows that it is possible to think outside the box and come up with something new or different using the tools available.

# How did you make this project maintainable, readable, and adaptable?
It is readable and maintainable due to apropriate commenting and annotations. I did my best to make sure anything I added had a comment or two explaining the purpose of the code, so that way if I, or perhaps anyone else went to mantain the project, it would make sense what each part of the code is doing and therefore easy to be modified based on my example.

I think these projects are adaptable due to their base program, running with GPIO Interrupt, as both were using the same base but did different things. It could be possible to adapt this to something else entirely.
And even for the mock thermostat project, it could technically be altered and adapted to work for an A/C unit instead of a heating unit. The same premise exists, but instead of needing the setpoint to be higher than the temperature, it would want it to be lower than the temperature.
