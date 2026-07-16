---
name: handoff
description: Oturum devir notunu (docs/handoff.md) bu oturumda alınan kararlar, değişen durum ve açık işlerle güncelle
disable-model-invocation: true
---

# Oturum Devir Notu Güncelleme

`docs/handoff.md` bu projenin oturumlar arası hafızasıdır: yeni bir Claude
oturumu onu okuyarak kaldığı yerden devam eder. Bu beceri çağrıldığında
dosyayı bu oturumun çıktılarıyla güncelle.

## Adımlar

1. `docs/handoff.md`'yi **tamamen oku** — mevcut yapıyı ve bölüm başlıklarını öğren.
2. Bu oturumu gözden geçir ve şunları çıkar:
   - **Alınan kararlar** — gerekçeleriyle birlikte ("Alınmış kararlar" bölümüne
     numaralandırmayı sürdürerek ekle)
   - **Değişen durum** — tamamlanan işler, yeni eklenen dosyalar/paketler,
     çözülen veya yeni keşfedilen riskler
   - **Açık işler / sıradaki adımlar** — yarım kalanlar, doğrulanması gerekenler
3. Dosyayı güncelle:
   - Mevcut bölüm yapısını ve Türkçe dilini **koru**; başlıkları yeniden adlandırma.
   - Var olan kararları **silme veya yeniden yazma** — geçersizleşen bir karar
     varsa üzerine "~~iptal~~: <neden>" notu düş.
   - Göreli tarihleri mutlak tarihe çevir ("bugün" → gerçek tarih).
   - "Kullanıcı Türkçe konuşuyor" notunu asla kaldırma.
4. Sonunda kullanıcıya hangi bölümlere ne eklendiğini kısaca özetle.

## Yapma

- Kod tabanından zaten okunabilecek ayrıntıları (dosya içerikleri, fonksiyon
  imzaları) kopyalama — dosya kararları ve durumu taşır, kodu değil.
- Bu oturumda konuşulmamış hiçbir şeyi tahminle ekleme.
