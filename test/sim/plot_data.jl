using Plots

ts = problem.task_data[:diagnostics][:t]
qs = problem.task_data[:diagnostics][:q]
qsdot = problem.task_data[:diagnostics][:qdot]
torques = problem.task_data[:diagnostics][:torques]

Plots.plot(ts, hcat(torques...)')

# Plots.plot(ts, hcat(qsdot...)')
# lhrq = [q[1] for q in qs]
# lhpq = [q[2] for q in qs]
# lhkq = [q[3] for q in qs]
# rhrq = [q[4] for q in qs]
# rhpq = [q[5] for q in qs]
# rhkq = [q[6] for q in qs]

# lhrqdot = [q[1] for q in qsdot]
# lhpqdot = [q[2] for q in qsdot]
# lhkqdot = [q[3] for q in qsdot]
# rhrqdot = [q[4] for q in qsdot]
# rhpqdot = [q[5] for q in qsdot]
# rhkqdot = [q[6] for q in qsdot]

# Plots.plot(ts, lhrq)
# savefig("figures/after/lhrq.png")
# Plots.plot(ts, lhpq)
# savefig("figures/after/lhpq.png")
# Plots.plot(ts, lhkq)
# savefig("figures/after/lhkq.png")
# Plots.plot(ts, rhrq)
# savefig("figures/after/rhrq.png")
# Plots.plot(ts, rhpq)
# savefig("figures/after/rhpq.png")
# Plots.plot(ts, rhkq)
# savefig("figures/after/rhkq.png")

# Plots.plot(ts, lhrqdot)
# savefig("figures/after/lhrqdot.png")
# Plots.plot(ts, lhpqdot)
# savefig("figures/after/lhpqdot.png")
# Plots.plot(ts, lhkqdot)
# savefig("figures/after/lhkqdot.png")
# Plots.plot(ts, rhrqdot)
# savefig("figures/after/rhrqdot.png")
# Plots.plot(ts, rhpqdot)
# savefig("figures/after/rhpqdot.png")
# Plots.plot(ts, rhkqdot)
# savefig("figures/after/rhkqdot.png")