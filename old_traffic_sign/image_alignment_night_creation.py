import cv2
import numpy as np
from scipy.ndimage import fourier_shift

# 이미지 로드
img_day = cv2.imread('no_flash_image.jpg', cv2.IMREAD_GRAYSCALE)
img_flash_day = cv2.imread('flash_day_image.jpg', cv2.IMREAD_GRAYSCALE)

# 전역 정렬 - FFT를 이용한 상관 관계
def global_align(img1, img2):
    f1 = np.fft.fft2(img1)
    f2 = np.fft.fft2(img2)
    shape = img1.shape
    cross_power_spectrum = (f1 * f2.conj()) / np.abs(f1 * f2.conj())
    shift = np.fft.ifft2(cross_power_spectrum)
    shift = np.fft.fftshift(shift)
    max_shift = np.unravel_index(np.argmax(np.abs(shift)), shape)
    shifts = np.array(max_shift) - np.array(shape) // 2
    aligned_img2 = np.real(np.fft.ifft2(fourier_shift(np.fft.fft2(img2), shifts)))
    return aligned_img2, shifts

aligned_img, shifts = global_align(img_day, img_flash_day)

# 지역 정렬 - 서브픽셀 정렬
def local_align(img1, img2, block_size=100):
    aligned_img = np.zeros_like(img2)
    for i in range(0, img1.shape[0], block_size):
        for j in range(0, img1.shape[1], block_size):
            block1 = img1[i:i+block_size, j:j+block_size]
            block2 = img2[i:i+block_size, j:j+block_size]
            f1 = np.fft.fft2(block1)
            f2 = np.fft.fft2(block2)
            cross_power_spectrum = (f1 * f2.conj()) / np.abs(f1 * f2.conj())
            shift = np.fft.ifft2(cross_power_spectrum)
            shift = np.fft.fftshift(shift)
            max_shift = np.unravel_index(np.argmax(np.abs(shift)), block1.shape)
            shifts = np.array(max_shift) - np.array(block1.shape) // 2
            aligned_block2 = np.real(np.fft.ifft2(fourier_shift(np.fft.fft2(block2), shifts)))
            aligned_img[i:i+block_size, j:j+block_size] = aligned_block2
    return aligned_img

aligned_img = local_align(aligned_img, aligned_img)

# 태양광 제거 및 야간 이미지 생성
night_img = img_flash_day - aligned_img

# 결과 저장
cv2.imwrite('aligned_image.jpg', aligned_img)
cv2.imwrite('night_image.jpg', night_img)

# 이미지 출력
cv2.imshow('Aligned Image', aligned_img)
cv2.imshow('Night Image', night_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
