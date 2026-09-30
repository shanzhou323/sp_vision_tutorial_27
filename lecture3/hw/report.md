# 项目理解报告

## 1. 图像生命周期与所有权
图像源模拟相机的可复用缓冲区：每次调用 next() 都把新图读进同一块 buffer_，
旧 buffer 会被新像素覆盖。cv::Mat 的普通赋值（=）是浅拷贝，只复制矩阵头和
指针，多个 Mat 共享同一块像素数据。如果入队的 frame.image 直接引用 buffer_，
下一次 next() 写入 buffer_ 时队列里的帧就被悄悄改掉了。

我把 frame.image 改成 buffer_.clone()，让每一帧在离开 next() 时拥有一块独立
的、自己的像素内存。后续 next() 再写 buffer_ 不会影响已入队的帧。

## 2. 并发处理与恰好一次
BlockingQueue 的 push 把帧放入队列，pop 从队列取出一个元素。多个 worker 同时
pop 时，队列内部互斥保证同一元素只被一个 worker 拿到，不会重复处理。
producer 读完所有图后调 close()，队列关闭后 pop 会在剩余元素取完后返回 false，
worker 自然退出循环。不会漏帧因为 close 后队列里残留的帧仍会被 pop 完；
不会重复因为 pop 是"取出"而不是"偷看"。输入耗尽时，正在等待的 worker 被
close() 唤醒后看到队列已空就退出；仍在处理某帧的 worker 处理完再 pop 时返回
false 退出。

## 3. 共享统计数据
producer 调 onProduced，每个 worker 调 onProcessed/onSaved/onCorrupted。
原实现 deliberatelySlowIncrement 先读值、sleep 100 微秒、再写回 +1，多线程下
两个线程可能同时读到同一个旧值，各自 +1 写回，结果只加了一次（丢更新）。
我给 Statistics 加了一把 std::mutex，所有 onXxx() 和 snapshot() 都用
lock_guard 保护同一把锁，保证自增是原子的。snapshot 也持锁，所以读到的四个
计数是一致的瞬间快照，不会出现 produced 加了但 processed 还没加完的中间态。

## 4. 线程关闭协议
路径1（显式 wait）：producer 读完图后调 queue_.close()。worker 把队列里剩余
帧处理完后 pop 返回 false 退出循环。wait() join producer，再逐个 join workers，
所有线程结束后 wait 返回，没有线程还在跑。

路径2（不调 wait 直接析构）：析构函数先调 queue_.close() 唤醒所有等待的
worker，然后 join producer 和所有 workers。即使此时 producer 还在读图，
它读完也会自然 close（重复 close 无害）；worker 把剩余帧处理完后退出。
join 保证析构返回时没有任何线程仍在运行，不会触发 std::terminate，也不会
访问已销毁的对象。