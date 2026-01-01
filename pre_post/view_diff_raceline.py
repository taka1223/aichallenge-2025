# Python の例（pandas を想定）
import matplotlib.pyplot as plt

# df: 目標と実測のログを time を index にして結合
plt.figure()
plt.plot(df['target_speed'], label='target_speed')
plt.plot(df['actual_speed'], label='actual_speed')
plt.legend()
plt.title('Speed: Target vs Actual')

plt.figure()
plt.plot(df['target_steer'], label='target_steer')
plt.plot(df['actual_steer'], label='actual_steer')
plt.legend()
plt.title('Steer: Target vs Actual')
