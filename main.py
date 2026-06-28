import time
import cv2
import mss
import numpy as np
import keyboard
import tkinter as tk
import os
import ctypes
from ctypes import wintypes

# =====================================================================
# 1. СТРУКТУРЫ WIN32 API ДЛЯ DIRECTINPUT (CTYPES)
# =====================================================================
# Включение DPI awareness, чтобы масштабирование Windows (125%, 150%) не ломало координаты захвата экрана
try:
    ctypes.windll.shcore.SetProcessDpiAwareness(2)
except Exception:
    try:
        ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

user32 = ctypes.WinDLL('user32', use_last_error=True)

# Скан-коды клавиш DirectInput
SCAN_A = 0x1E
SCAN_D = 0x20
SCAN_SPACE = 0x39

INPUT_KEYBOARD = 1
KEYEVENTF_SCANCODE = 0x0008
KEYEVENTF_KEYUP = 0x0002

ULONG_PTR = ctypes.c_ulong if ctypes.sizeof(ctypes.c_void_p) == 4 else ctypes.c_uint64

class KEYBDINPUT(ctypes.Structure):
    _fields_ = [
        ("wVk", wintypes.WORD),
        ("wScan", wintypes.WORD),
        ("dwFlags", wintypes.DWORD),
        ("time", wintypes.DWORD),
        ("dwExtraInfo", ULONG_PTR)
    ]

class MOUSEINPUT(ctypes.Structure):
    _fields_ = [
        ("dx", wintypes.LONG),
        ("dy", wintypes.LONG),
        ("mouseData", wintypes.DWORD),
        ("dwFlags", wintypes.DWORD),
        ("time", wintypes.DWORD),
        ("dwExtraInfo", ULONG_PTR)
    ]

class HARDWAREINPUT(ctypes.Structure):
    _fields_ = [
        ("uMsg", wintypes.DWORD),
        ("wParamL", wintypes.WORD),
        ("wParamH", wintypes.WORD)
    ]

class INPUT_UNION(ctypes.Union):
    _fields_ = [
        ("ki", KEYBDINPUT),
        ("mi", MOUSEINPUT),
        ("hi", HARDWAREINPUT)
    ]

class INPUT(ctypes.Structure):
    _fields_ = [
        ("type", wintypes.DWORD),
        ("ii", INPUT_UNION)
    ]

def press_key_direct(scancode):
    """Отправка нажатия клавиши через DirectInput"""
    ii_ = INPUT_UNION()
    ii_.ki = KEYBDINPUT(0, scancode, KEYEVENTF_SCANCODE, 0, 0)
    x = INPUT(INPUT_KEYBOARD, ii_)
    user32.SendInput(1, ctypes.byref(x), ctypes.sizeof(x))

def release_key_direct(scancode):
    """Отправка отпускания клавиши через DirectInput"""
    ii_ = INPUT_UNION()
    ii_.ki = KEYBDINPUT(0, scancode, KEYEVENTF_SCANCODE | KEYEVENTF_KEYUP, 0, 0)
    x = INPUT(INPUT_KEYBOARD, ii_)
    user32.SendInput(1, ctypes.byref(x), ctypes.sizeof(x))

# =====================================================================
# 2. НАСТРОЙКА ОБЛАСТИ ЭКРАНА И ПАРАМЕТРОВ
# =====================================================================
MONITOR_ZONE = {"top": 156, "left": 648, "width": 623, "height": 847}

# Диапазон цвета для ботинка (HSV)
LOWER_BOOT = np.array([2, 145, 40])
UPPER_BOOT = np.array([12, 255, 120])

BOOT_MIN_X = 15   # Расширили границы, чтобы точнее ловить сапог у левой стены
BOOT_MAX_X = 615  # Расширили границы, чтобы точнее ловить сапог у правой стены
BOOT_MIN_AREA = 15
BOOT_MAX_AREA = 1000

CART_TARGET_Y = 720
CART_CENTER_OFFSET = -12  # Офсет корректировки центра платформы влево

# =====================================================================
# ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ И СОСТОЯНИЯ
# =====================================================================
boot_history = []
movement_state = 'IDLE'  # Физическое состояние: 'LEFT', 'RIGHT', 'IDLE'
smoothed_target_x = None
lost_boot_frames = 0
frame_count = 0
last_space_press_time = 0

# Память положения каретки для исключения потери фокуса
last_known_cart_x = 311  
lost_cart_frames = 0

# Кэш координат для высокоскоростного ROI-трекинга сапога
last_boot_x = None
last_boot_y = None

# Физические векторы для фильтрации вращения сапога
last_valid_dy = 0.0
last_valid_dx = 0.0

# =====================================================================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ВЫЧИСЛЕНИЙ И ДЕТЕКЦИИ
# =====================================================================

def reflect_x(x, left=30, right=593):
    """
    Математически точный расчет отскока от боковых стен.
    Переводит координату x в рамки [left, right], симулируя упругое столкновение.
    """
    width = right - left
    x_shifted = x - left
    cycle = 2 * width
    x_mod = x_shifted % cycle
    if x_mod > width:
        return right - (x_mod - width)
    else:
        return left + x_mod

def find_boot_coords(bgr_slice):
    """Поиск ботинка по его цветовой маске с динамической оптимизацией ROI"""
    global last_boot_x, last_boot_y
    
    use_roi = False
    roi_x_start, roi_y_start = 0, 0
    
    # Высокоскоростной трекинг: обрезаем картинку вокруг прошлой позиции сапога
    if last_boot_x is not None and last_boot_y is not None:
        pad_x, pad_y = 120, 120  # Достаточный запас под скорость падения
        roi_x_start = max(30, last_boot_x - pad_x)
        roi_x_end = min(622, last_boot_x + pad_x)
        roi_y_start = max(0, last_boot_y - pad_y)
        roi_y_end = min(740, last_boot_y + pad_y)
        
        if roi_x_end > roi_x_start and roi_y_end > roi_y_start:
            bgr_boot = bgr_slice[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
            use_roi = True
            
    if not use_roi:
        # Полный кадр поиска при первой детекции или утере
        roi_x_start = 30
        roi_y_start = 0
        bgr_boot = bgr_slice[:740, 30:622]
        
    hsv_img = cv2.cvtColor(bgr_boot, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, LOWER_BOOT, UPPER_BOOT)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_contour = None
    best_area = -1
    best_cx, best_cy = 0, 0
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if BOOT_MIN_AREA <= area <= BOOT_MAX_AREA:
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w // 2 + roi_x_start  
            cy = y + h // 2 + roi_y_start
            if BOOT_MIN_X <= cx <= BOOT_MAX_X:
                if area > best_area:
                    best_area = area
                    best_contour = contour
                    best_cx, best_cy = cx, cy
            
    if best_contour is not None:
        last_boot_x, last_boot_y = best_cx, best_cy
        return best_cx, best_cy, best_area
        
    # Если сапог совершил резкий рывок и вышел за границы ROI — сбрасываем кэш и ищем по всей площади
    if use_roi:
        last_boot_x, last_boot_y = None, None
        return find_boot_coords(bgr_slice)
        
    return None

def find_cart_coords(bgr_slice):
    """Поиск центра каретки по золотым элементам с корректировкой смещения"""
    slice_y_start = 795
    slice_x_start = 30
    img_strip = bgr_slice[slice_y_start:830, slice_x_start:622]
    
    # Детекция золотого/желтого цвета в BGR
    gold_mask = (img_strip[:, :, 2] > 120) & (img_strip[:, :, 1] > 80) & (img_strip[:, :, 0] < 90) & (img_strip[:, :, 2] > img_strip[:, :, 1])
    
    y_indices, x_indices = np.nonzero(gold_mask)
    if len(x_indices) > 20:  
        cart_x = int(np.mean(x_indices)) + slice_x_start + CART_CENTER_OFFSET
        cart_y = 727
        return cart_x, cart_y, len(x_indices), "GOLD_CENTER"
        
    return None

def set_movement(target_state):
    """Переключатель направления движения через DirectInput (ctypes)"""
    global movement_state
    if movement_state == target_state:
        return
        
    if movement_state == 'LEFT':
        release_key_direct(SCAN_A)
    elif movement_state == 'RIGHT':
        release_key_direct(SCAN_D)
        
    if target_state == 'LEFT':
        press_key_direct(SCAN_A)
    elif target_state == 'RIGHT':
        press_key_direct(SCAN_D)
        
    movement_state = target_state

def stop_movement():
    set_movement('IDLE')

def press_space_safe():
    """Нажатие пробела через DirectInput (ctypes) с задержкой удержания"""
    press_key_direct(SCAN_SPACE)
    time.sleep(0.05)
    release_key_direct(SCAN_SPACE)

# =====================================================================
# НАСТРОЙКА ИНТЕРФЕЙСА ОВЕРЛЕЯ (TKINTER)
# =====================================================================
root = tk.Tk()
root.overrideredirect(True)        
root.attributes("-topmost", True)  
root.attributes("-transparentcolor", "black")
root.config(bg="black")

geom = f"{MONITOR_ZONE['width']}x{MONITOR_ZONE['height']}+{MONITOR_ZONE['left']}+{MONITOR_ZONE['top']}"
root.geometry(geom)

canvas = tk.Canvas(root, bg="black", highlightthickness=0)
canvas.pack(fill="both", expand=True)

sct = mss.MSS()

# Элементы HUD
hud_bg = canvas.create_rectangle(15, 15, 250, 230, fill="#09090b", outline="#27272a", width=1)
hud_title = canvas.create_text(30, 30, text="BOOT BREAKER PRO", fill="#f4f4f5", anchor="nw", font=("Segoe UI", 9, "bold"))
hud_line1 = canvas.create_line(30, 48, 235, 48, fill="#27272a", width=1)
hud_status = canvas.create_text(30, 58, text="● ACTIVE", fill="#10b981", anchor="nw", font=("Segoe UI", 9, "bold"))
hud_boot_lbl = canvas.create_text(30, 80, text="BOOT DETECTOR", fill="#a1a1aa", anchor="nw", font=("Segoe UI", 8, "bold"))
hud_boot_val = canvas.create_text(30, 95, text="NOT FOUND", fill="#f43f5e", anchor="nw", font=("Segoe UI", 9, "bold"))
hud_boot_pct = canvas.create_text(30, 110, text="", fill="#fb923c", anchor="nw", font=("Segoe UI", 8))
hud_line2 = canvas.create_line(30, 128, 235, 128, fill="#27272a", width=1)
hud_cart_lbl = canvas.create_text(30, 138, text="CART DETECTOR", fill="#a1a1aa", anchor="nw", font=("Segoe UI", 8, "bold"))
hud_cart_val = canvas.create_text(30, 153, text="NOT FOUND", fill="#f43f5e", anchor="nw", font=("Segoe UI", 9, "bold"))
hud_line3 = canvas.create_line(30, 175, 235, 175, fill="#27272a", width=1)
hud_pred_lbl = canvas.create_text(30, 185, text="PREDICTION", fill="#a1a1aa", anchor="nw", font=("Segoe UI", 8, "bold"))
hud_pred_val = canvas.create_text(30, 202, text="Awaiting tracking...", fill="#71717a", anchor="nw", font=("Segoe UI", 8, "italic"))

marker_border = canvas.create_rectangle(1, 1, MONITOR_ZONE["width"]-1, MONITOR_ZONE["height"]-1, outline="#3f3f46", width=1)
marker_boot = canvas.create_oval(0, 0, 0, 0, outline="#a78bfa", width=2, fill="#a78bfa", state="hidden")
marker_boot_txt = canvas.create_text(0, 0, text="", fill="#a78bfa", anchor="w", font=("Segoe UI", 8, "bold"), state="hidden")
marker_cart = canvas.create_oval(0, 0, 0, 0, outline="#10b981", width=2, fill="#10b981", state="hidden")
marker_cart_txt = canvas.create_text(0, 0, text="", fill="#10b981", anchor="w", font=("Segoe UI", 8, "bold"), state="hidden")
marker_line = canvas.create_line(0, 0, 0, 0, fill="#f59e0b", width=1, dash=(4, 4), state="hidden")
marker_pred_pt = canvas.create_oval(0, 0, 0, 0, outline="#fb923c", width=2, fill="#fb923c", state="hidden")
marker_traj = canvas.create_line(0, 0, 0, 0, fill="#a78bfa", width=1.5, dash=(4, 4), state="hidden")

# Первичный рендеринг и биндинг Win32 API свойств прозрачности и скрытия от захвата экрана
root.update()
try:
    hwnd = int(root.wm_frame(), 16)
    if hwnd:
        # Установка флагов сквозного клика (Click-Through) на главное окно
        GWL_EXSTYLE = -20
        WS_EX_TRANSPARENT = 0x00000020
        style = user32.GetWindowLongW(hwnd, GWL_EXSTYLE)
        user32.SetWindowLongW(hwnd, GWL_EXSTYLE, style | WS_EX_TRANSPARENT)
        
        # Полное исключение оверлея из захвата mss / Discord (сапог больше не теряется под окном)
        WDA_EXCLUDEFROMCAPTURE = 0x00000011
        user32.SetWindowDisplayAffinity(hwnd, WDA_EXCLUDEFROMCAPTURE)
except Exception as e:
    print(f"[-] Ошибка инициализации Win32 стилей: {e}")

def draw_shadcn_hud(boot_det, bx, by, ba, pct, cart_det, cx, cy, method, action, px, diff, paused=False):
    if paused:
        canvas.itemconfigure(hud_status, text="● PAUSED (Manual)", fill="#fb923c")
        canvas.itemconfigure(hud_boot_val, text="Shift holds script", fill="#71717a")
        canvas.itemconfigure(hud_boot_pct, text="")
        canvas.itemconfigure(hud_cart_val, text="")
        canvas.itemconfigure(hud_pred_val, text="")
        return

    canvas.itemconfigure(hud_status, text="● ACTIVE", fill="#10b981")
    if boot_det:
        canvas.itemconfigure(hud_boot_val, text=f"X: {bx}  Y: {by} | {ba:.0f}px", fill="#e4e4e7")
        canvas.itemconfigure(hud_boot_pct, text=f"Approach: {pct:.1f}%")
    else:
        canvas.itemconfigure(hud_boot_val, text="NOT FOUND", fill="#f43f5e")
        canvas.itemconfigure(hud_boot_pct, text="")

    if cart_det:
        canvas.itemconfigure(hud_cart_val, text=f"X: {cx}  Y: {cy} | {method}", fill="#e4e4e7")
    else:
        canvas.itemconfigure(hud_cart_val, text="NOT FOUND", fill="#f43f5e")

    if boot_det and cart_det:
        diff_str = f"+{diff}" if diff >= 0 else f"{diff}"
        canvas.itemconfigure(hud_pred_val, text=f"Target X: {px} ({diff_str}px) | {action}", fill="#f4f4f5")
    else:
        canvas.itemconfigure(hud_pred_val, text="Awaiting tracking...", fill="#71717a")

# =====================================================================
# ОСНОВНОЙ ВЫСОКОСКОРОСТНОЙ ЦИКЛ БОТА
# =====================================================================
def tick_overlay():
    global boot_history, smoothed_target_x, lost_boot_frames, frame_count, last_space_press_time
    global last_known_cart_x, lost_cart_frames, last_valid_dy, last_valid_dx

    # Ручной перехват управления (удержание Shift)
    if keyboard.is_pressed('shift'):
        stop_movement()
        boot_history.clear()
        smoothed_target_x = None
        lost_boot_frames = 0
        lost_cart_frames = 0
        
        canvas.itemconfigure(marker_border, outline="#f59e0b")
        canvas.itemconfigure(marker_boot, state="hidden")
        canvas.itemconfigure(marker_boot_txt, state="hidden")
        canvas.itemconfigure(marker_cart, state="hidden")
        canvas.itemconfigure(marker_cart_txt, state="hidden")
        canvas.itemconfigure(marker_line, state="hidden")
        canvas.itemconfigure(marker_pred_pt, state="hidden")
        canvas.itemconfigure(marker_traj, state="hidden")
        
        draw_shadcn_hud(False, 0, 0, 0, 0, False, 0, 0, "", "", 0, 0, paused=True)
        return

    # 1. Сверхбыстрый захват экрана (в буфер не попадает оверлей)
    img = sct.grab(MONITOR_ZONE)
    bgra = np.frombuffer(img.raw, dtype=np.uint8).reshape((img.height, img.width, 4))
    bgr_slice = bgra[:830, :, :3]  

    # Детекция объектов
    boot_data = find_boot_coords(bgr_slice)
    cart_data = find_cart_coords(bgr_slice)

    boot_detected = boot_data is not None
    cart_detected = cart_data is not None

    # Обработка амортизации потери каретки
    if cart_detected:
        cart_x, cart_y, cart_area, cart_method = cart_data
        last_known_cart_x = cart_x
        lost_cart_frames = 0
    else:
        lost_cart_frames += 1
        if lost_cart_frames < 8 and last_known_cart_x is not None:
            cart_x = last_known_cart_x
            cart_y = 727
            cart_area = 0
            cart_method = "MEM_FALLBACK"
            cart_detected = True
        else:
            cart_x, cart_y, cart_area, cart_method = 0, 0, 0, ""

    # Автоматический запуск раундов
    is_start_screen = False
    try:
        h_slice, w_slice, _ = bgr_slice.shape
        if h_slice > 513 and w_slice > 197:
            p512 = bgr_slice[512, 197]
            p513 = bgr_slice[513, 197]
            p511 = bgr_slice[511, 197]
            p510 = bgr_slice[510, 197]
            
            is_white_512 = all(215 <= c <= 250 for c in p512)
            is_white_513 = all(215 <= c <= 250 for c in p513)
            is_dark_511 = (45 <= p511[0] <= 70) and (38 <= p511[1] <= 60) and (30 <= p511[2] <= 52)
            is_dark_510 = (45 <= p510[0] <= 70) and (38 <= p510[1] <= 60) and (30 <= p510[2] <= 52)
            
            if is_white_512 and is_white_513 and is_dark_511 and is_dark_510:
                is_start_screen = True
    except IndexError:
        pass

    if is_start_screen:
        current_time = time.time()
        if current_time - last_space_press_time > 2.5:
            press_space_safe()
            time.sleep(0.2)
            press_space_safe()
            last_space_press_time = current_time

    boot_x, boot_y, boot_area = boot_data if boot_detected else (0, 0, 0)

    predicted_x = boot_x
    diff = 0
    pct_approach = 0.0
    action = "IDLE"
    dx = 0.0
    dy = 0

    canvas.itemconfigure(marker_border, outline="#3f3f46")

    if boot_detected and cart_detected:
        lost_boot_frames = 0
        pct_approach = min(100.0, max(0.0, (boot_y / CART_TARGET_Y) * 100))

        # Накопление истории траектории
        boot_history.append((boot_x, boot_y))
        if len(boot_history) > 8:
            boot_history.pop(0)

        # --- СВЕРХБЫСТРОЕ ПРЕДСКАЗАНИЕ ОТСКОКА ---
        if len(boot_history) >= 3:
            dx_now = boot_history[-1][0] - boot_history[-2][0]
            dy_now = boot_history[-1][1] - boot_history[-2][1]
            dx_prev = boot_history[-2][0] - boot_history[-3][0]
            dy_prev = boot_history[-2][1] - boot_history[-3][1]
            
            bounce_x = (dx_now * dx_prev < 0) and (abs(dx_now) > 1.5 or abs(dx_prev) > 1.5)
            bounce_y = (dy_now * dy_prev < 0) and (abs(dy_now) > 1.5 or abs(dy_prev) > 1.5)
            
            if bounce_x:
                last_p = boot_history[-1]
                synthetic_prev = (last_p[0] + dx_prev, last_p[1] - dy_now)
                boot_history = [synthetic_prev, last_p]
            elif bounce_y:
                boot_history = [boot_history[-1]]
                smoothed_target_x = None

        if len(boot_history) >= 2:
            p_first = boot_history[0]
            p_last = boot_history[-1]
            frames_span = len(boot_history) - 1
            
            dx = (p_last[0] - p_first[0]) / frames_span
            dy = (p_last[1] - p_first[1]) / frames_span

            # --- ФИЛЬТРАЦИЯ ДВУХОСЕВОГО ВРАЩЕНИЯ САПОГА ---
            if dy < 1.0 and boot_y > 150:
                if last_valid_dy > 1.0:
                    dy = last_valid_dy  # Игнорируем ложное смещение вверх при вращении сапога
            else:
                last_valid_dy = dy

            if last_valid_dx != 0.0 and abs(dx - last_valid_dx) > 15.0:
                dx = 0.7 * last_valid_dx + 0.3 * dx  # Сглаживаем аномальные горизонтальные рывки детекции
            last_valid_dx = dx

            # --- СМЕШИВАНИЕ ЦЕЛЕЙ И ОТРАЖЕНИЕ ОТ СТЕН ---
            if dy > 0.5:  
                distance_y = CART_TARGET_Y - boot_y
                steps = min(120.0, max(0.0, distance_y / dy))
                raw_predicted_x = boot_x + dx * steps
                
                # Применение точной симуляции отскока от стен
                proj_x = reflect_x(raw_predicted_x, left=30, right=593)
                
                # Плавное увеличение веса предсказания по мере приближения к платформе (без разрывов)
                if boot_y < 450:
                    blend = 0.2 + 0.8 * (boot_y / 450.0)
                else:
                    blend = 1.0  # На критической высоте полностью доверяем предсказанию траектории
                
                predicted_x = int(boot_x * (1.0 - blend) + proj_x * blend)
            else:  
                predicted_x = boot_x

        # --- ДИНАМИЧЕСКОЕ СГЛАЖИВАНИЕ ФИЛЬТРОМ EMA ---
        if smoothed_target_x is None:
            smoothed_target_x = predicted_x
        else:
            if boot_y > 550:
                alpha = 0.95  # Почти мгновенная реакция перед поимкой
            elif boot_y > 350:
                alpha = 0.70
            else:
                alpha = 0.40
            smoothed_target_x = alpha * predicted_x + (1.0 - alpha) * smoothed_target_x
            
        final_target_x = int(smoothed_target_x)
        diff = final_target_x - cart_x

        # --- ДИНАМИЧЕСКИЙ ГИСТЕРЕЗИС И ДОВОДКА НА КРАЯХ ЭКРАНА ---
        if boot_y > 550:
            deadzone_stop = 2    
            deadzone_start = 4   
        elif boot_y > 350:
            deadzone_stop = 3
            deadzone_start = 7
        else:
            deadzone_stop = 5    
            deadzone_start = 12

        # Если цель находится близко к границам экрана — убираем мертвую зону
        if final_target_x < 60 or final_target_x > 560:
            deadzone_stop = 1
            deadzone_start = 2

        abs_diff = abs(diff)
        
        # --- ДВУХПОРОГОВЫЙ СУПЕРСТАБИЛЬНЫЙ КОНТРОЛЛЕР ---
        if movement_state == 'IDLE':
            if abs_diff > deadzone_start:
                if diff > 0:
                    set_movement('RIGHT')
                    action = "RIGHT"
                else:
                    set_movement('LEFT')
                    action = "LEFT"
            else:
                action = "ALIGN"
        else:
            # Предотвращаем микро-остановки (stuttering) при погоне за быстрым сапогом
            should_stop = abs_diff < deadzone_stop
            if should_stop and abs(dx) > 1.0:
                # Если сапог летит быстро и мы всё еще движемся в его сторону — продолжаем погоню
                if (movement_state == 'RIGHT' and diff > 0) or (movement_state == 'LEFT' and diff < 0):
                    should_stop = False

            if should_stop:
                stop_movement()
                action = "ALIGN"
            elif (diff > 0 and movement_state == 'LEFT') or (diff < 0 and movement_state == 'RIGHT'):
                if diff > 0:
                    set_movement('RIGHT')
                    action = "REVERSE_RIGHT"
                else:
                    set_movement('LEFT')
                    action = "REVERSE_LEFT"
            else:
                action = movement_state

        # Обновление отрисовки траекторий
        canvas.coords(marker_line, 0, CART_TARGET_Y, MONITOR_ZONE["width"], CART_TARGET_Y)
        canvas.itemconfigure(marker_line, state="normal")
        
        canvas.coords(marker_pred_pt, final_target_x-4, CART_TARGET_Y-4, final_target_x+4, CART_TARGET_Y+4)
        canvas.itemconfigure(marker_pred_pt, state="normal")
        
        canvas.coords(marker_traj, boot_x, boot_y, final_target_x, CART_TARGET_Y)
        canvas.itemconfigure(marker_traj, state="normal")
    else:
        if lost_boot_frames < 3 and len(boot_history) > 0:
            lost_boot_frames += 1
        else:
            stop_movement()
            boot_history.clear()
            smoothed_target_x = None
            action = "WAIT"
            
            canvas.itemconfigure(marker_line, state="hidden")
            canvas.itemconfigure(marker_pred_pt, state="hidden")
            canvas.itemconfigure(marker_traj, state="hidden")

    # Отображение точечных центров объектов в реальном времени
    if boot_detected:
        canvas.coords(marker_boot, boot_x-4, boot_y-4, boot_x+4, boot_y+4)
        canvas.coords(marker_boot_txt, boot_x+10, boot_y)
        canvas.itemconfigure(marker_boot_txt, text=f"({boot_x},{boot_y})")
        canvas.itemconfigure(marker_boot, state="normal")
        canvas.itemconfigure(marker_boot_txt, state="normal")
    else:
        canvas.itemconfigure(marker_boot, state="hidden")
        canvas.itemconfigure(marker_boot_txt, state="hidden")

    if cart_detected:
        canvas.coords(marker_cart, cart_x-4, cart_y-4, cart_x+4, cart_y+4)
        canvas.coords(marker_cart_txt, cart_x+10, cart_y)
        canvas.itemconfigure(marker_cart_txt, text=f"({cart_x},{cart_y})")
        canvas.itemconfigure(marker_cart, state="normal")
        canvas.itemconfigure(marker_cart_txt, state="normal")
    else:
        canvas.itemconfigure(marker_cart, state="hidden")
        canvas.itemconfigure(marker_cart_txt, state="hidden")

    # Тяжелая отрисовка текста HUD - раз в 12 кадров (разгрузка CPU)
    frame_count += 1
    if frame_count % 12 == 0:
        draw_shadcn_hud(boot_detected, boot_x, boot_y, boot_area, pct_approach,
                        cart_detected, cart_x, cart_y, cart_method, action, int(smoothed_target_x) if smoothed_target_x is not None else boot_x, diff)

def on_escape(event):
    stop_movement()
    root.destroy()

root.bind("<Escape>", on_escape)

# =====================================================================
# ВЫСОКОСКОРОСТНОЙ ЦИКЛ ОБРАБОТКИ С ОГРАНИЧЕНИЕМ КАДРОВ
# =====================================================================
try:
    print("="*60)
    print("[+] BOOT BREAKER PRO ЗАПУЩЕН!")
    print("="*60)
    print("[!] Важно: Игра должна быть в разрешении 1920x1080 (без рамки или во весь экран).")
    print("[!] Терминал ДОЛЖЕН быть запущен от ИМЕНИ АДМИНИСТРАТОРА,")
    print("    иначе Dota 2 заблокирует симуляцию нажатий клавиш.")
    print("[!] Управление:")
    print("    - Удерживайте [SHIFT] для паузы (ручной контроль)")
    print("    - Нажмите [ESC] (кликнув на оверлей), чтобы закрыть скрипт")
    print("="*60)
    print("[~] Ожидание активности... Переключитесь на Dota 2.")
    
    # Ограничение FPS для плавной синхронизации и исключения фризов Discord
    TARGET_FPS = 120
    FRAME_BUDGET = 1.0 / TARGET_FPS
    
    while True:
        start_time = time.perf_counter()
        
        # Обновляем окно Tkinter (однократный вызов update() без дублирования idletasks)
        root.update()
        
        tick_overlay()
        
        elapsed = time.perf_counter() - start_time
        sleep_time = FRAME_BUDGET - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            time.sleep(0.0005)  # Разгрузка ядер процессора при тяжелых сценах
            
except (KeyboardInterrupt, tk.TclError):
    stop_movement()
    try:
        root.destroy()
    except Exception:
        pass
    print("\n[+] Скрипт завершен.")
    os._exit(0)
