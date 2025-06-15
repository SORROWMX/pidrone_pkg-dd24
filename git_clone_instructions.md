# Инструкции по клонированию репозитория pidrone_pkg-dd24

## Метод 1: Клонирование всего репозитория и переключение на нужную ветку

```bash
# Клонировать весь репозиторий
git clone https://github.com/SORROWMX/pidrone_pkg-dd24.git

# Перейти в директорию репозитория
cd pidrone_pkg-dd24

# Переключиться на ветку rollback_branch
git checkout rollback_branch
```

## Метод 2: Клонирование только конкретной ветки

```bash
# Клонировать только ветку rollback_branch
git clone -b rollback_branch --single-branch https://github.com/SORROWMX/pidrone_pkg-dd24.git

# Перейти в директорию репозитория
cd pidrone_pkg-dd24
```

Второй метод позволяет сэкономить время и место на диске, так как скачивается только нужная ветка. 