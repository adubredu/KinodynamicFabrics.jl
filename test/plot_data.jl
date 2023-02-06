using Plots 

p_com = problem.task_data[:diagnostics][:p_com]
p_zmp = problem.task_data[:diagnostics][:p_zmp]
ts = problem.task_data[:diagnostics][:t]
ns = problem.task_data[:diagnostics][:norm]

# Plots.plot([p[1] for p in p_com], [p[2] for p in p_com])
# savefig("com_plot.png")
# Plots.plot([p[1] for p in p_zmp], [p[2] for p in p_zmp] )
# savefig("zmp_plot.png")

Plots.plot(ts, ns)
savefig("norm.png")

Plots.plot(ts[1000:end], [p[1] for p in p_com[1000:end]])
savefig("com_time_plot.png")
Plots.plot(ts[1000:end], [p[1] for p in p_zmp[1000:end]])
savefig("zmp_time_plot.png")
