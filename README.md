# Electric_Bikes_Battery_Swapping
## 10/14 遗留问题
- [ ] 是否要考虑缩短banlist的长度？目前为无穷大。

- [ ] 对于每个neighbor搜索方式，记无改善次数是否应该清0。应该在什么时候记。

- [ ] 对于每次大的迭代。记无改善次数是否应该清0。

- [ ] build_initial_solution中，是否应该引入banlist，在插入depot点失败后尝试新的点。

- [ ] Routebuilder的目标函数需要更改与测试。

- [ ] 创建rule。

- [X] Routebuilder的get_feasibility()需要更改。

- [X] Routebuilder的build_initial_solution需要更改获取可行性方面的函数。

- [ ] 解决时刻为0时没有routed result的问题。

