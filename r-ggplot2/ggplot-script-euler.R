library("ggplot2")
setwd("D:/tmp-cartpole")

dat1 <- read.table("theta-RK1-tau0_001.csv", sep=",", head=TRUE)
dat2 <- read.table("theta-RK1-tau0_005.csv", sep=",", head=TRUE)
dat3 <- read.table("theta-RK1-tau0_01.csv", sep=",", head=TRUE)

#ggplot(data=dat2, aes(x=time, y=theta, group=1)) + geom_line()

p = ggplot() +
  geom_line(data=dat1, aes(x=time, y=theta, color="0.001")) +
  geom_line(data=dat2, aes(x=time, y=theta, color="0.005")) +
  geom_line(data=dat3, aes(x=time, y=theta, color="0.01")) +
  scale_color_manual(name = "Timestep(seconds)", values = c("0.001"="#D55E00", "0.005"="#0072B2", "0.01"="#56B4E9")) +
  theme(legend.position="top") +
  xlab("time (seconds)") +
  ylab("pole angle (radians)")

ggsave(filename="eulers-method.png", plot=p, height=4, width=7, units="in", dpi=100, type="cairo")


