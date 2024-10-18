import asyncio
from typing import Callable, List

class AsyncTimer:
    def __init__(self, interval: float, loop: asyncio.AbstractEventLoop):
        """
        Создает асинхронный таймер.

        :param interval: Интервал времени в секундах между вызовами колбэков.
        :param loop: Цикл событий, в котором будет работать таймер.
        """
        self.interval = interval
        self.loop = loop
        self.callbacks: List[Callable[[], None]] = []
        self._task = None
        self._running = False

    def subscribe(self, callback: Callable[[], None]) -> None:
        """
        Добавляет колбэк в список для вызова.

        :param callback: Функция-колбэк, которая будет вызвана по таймеру.
        """
        self.callbacks.append(callback)

    async def _run(self):
        """
        Запускает таймер и вызывает колбэки через заданные интервалы времени.
        """
        while self._running:
            await asyncio.sleep(self.interval)
            for callback in self.callbacks:
                callback()
    def getInterval(self):
        return self.interval

    def start(self) -> None:
        """
        Запускает выполнение таймера.
        """
        if not self._running:
            self._running = True
            self._task = self.loop.create_task(self._run())

    def stop(self) -> None:
        """
        Останавливает выполнение таймера.
        """
        if self._running:
            self._running = False
            if self._task:
                self._task.cancel()
                self._task = None