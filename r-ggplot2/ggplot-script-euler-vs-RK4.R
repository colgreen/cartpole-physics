library("ggplot2")
setwd("D:/tmp-cartpole")

dat1 <- read.table("theta-RK1-tau0_001.csv", sep=",", head=TRUE)
dat2 <- read.table("theta-RK4-tau0_001.csv", sep=",", head=TRUE)

#ggplot(data=dat2, aes(x=time, y=theta, group=1)) + geom_line()

p = ggplot() +
  geom_line(data=dat1, aes(x=time, y=theta, color="Euler")) +
  geom_line(data=dat2, aes(x=time, y=theta, color="RK4")) +
  scale_color_manual(name = "Timestep(seconds)", values = c("Euler"="#D55E00", "RK4"="#0072B2")) +
  theme(legend.position="top") +
  xlab("time (seconds)") +
  ylab("pole angle (radians)")

ggsave(filename="Euler-vs-RK4.png", plot=p, height=4, width=7, units="in", dpi=100, type="cairo")


